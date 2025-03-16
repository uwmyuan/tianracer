"""

Path tracking simulation with iterative linear model predictive control for speed and steer control
速度和转向控制的迭代线性模型预测控制算法及路径跟踪仿真

author: Atsushi Sakai (@Atsushi_twi)
revised: Yun Yuan (@uwmyuan)
"""
from ast import Return
from re import A
from statistics import variance
import cvxpy
import math
import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

# from tf_conversions import transformations
# import tf

try:
    import cubic_spline_planner  # 这里以三次样条曲线路径规划器为例，也可以更换其他规划算法
except:
    raise

NX = 4  # x = x, y, v, yaw 状态空间维度 包括x轴 y轴 线速度 偏航（朝向）
NU = 2  # a = [accel, steer]  控制空间维度 包括加速度和加角速度
T = 5  # horizon length 预测时间窗宽度
DT = 0.1  # [s] time tick 时钟单位

# mpc parameters 模型预测控制参数
GOAL_DIS = 1.5  # goal distance 与目标点的距离阈值 m
STOP_SPEED = 0.5 / 3.6  # stop speed 停止速度 除以3.6单位转化为m/s

# iterative paramter 迭代终止条件参数
MAX_ITER = 3  # Max iteration 最大迭代次数
DU_TH = 0.1  # iteration finish param 连续两次结果之差的阈值

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed 巡航速度 单位m/s
N_IND_SEARCH = 10  # Search index number 搜索最近航迹点的范围

ROSRATE = 10


class State:
    """
    vehicle state class
    车辆状态类
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, s=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.steer = s
        self.predelta = None


class Action:
    """
    vehicle action class
    车辆行为类
    """

    def __init__(self, acc=0.0, steer=0.0, vel=0.0):
        self.acc = acc
        self.vel = vel
        self.steer = steer


def pi_2_pi(angle):
    """
    rescale angle within a range [-pi, pi]
    """
    while angle > math.pi:
        angle = angle - 2.0 * math.pi

    while angle < -math.pi:
        angle = angle + 2.0 * math.pi

    return angle


def get_linear_model_matrix(v, phi, delta):
    """
    阿克曼车辆动力学模型的矩阵形式
    """
    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = -DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = -DT * v * math.cos(phi) * phi
    C[3] = -DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C


def update_state(state, a, delta):
    """
    predict the future state with the ackermann model
    更新状态
    """

    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    state.x = state.x + state.v * math.cos(state.yaw) * DT
    state.y = state.y + state.v * math.sin(state.yaw) * DT
    state.yaw = state.yaw + state.v / WB * math.tan(delta) * DT
    state.v = state.v + a * DT

    if state.v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED

    return state


def get_nparray_from_matrix(x):
    """
    a short rename from matrix flatten
    """
    return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind):
    """
    find the index of the nearest waypoint
    找到最近的航迹点序号
    """

    dx = [state.x - icx for icx in cx[pind : (pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind : (pind + N_IND_SEARCH)]]

    d = [idx**2 + idy**2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def predict_motion(x0, oa, od, xref):
    """
    predict the future states in the time horizon
    计算当前时间窗车辆状态
    """
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (ai, di, i) in zip(oa, od, range(1, T + 1)):
        state = update_state(state, ai, di)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar


def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    MPC contorl with updating operational point iteraitvely
    迭代更新操作点
    """

    if oa is None or od is None:
        oa = [0.0] * T
        od = [0.0] * T

    for _ in range(MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref)
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)
        # calc u change value
        du = sum(abs(oa - poa)) + sum(abs(od - pod))  
        if du <= DU_TH:
            break
    else:
        print("Iterative is max iter")

    return oa, od, ox, oy, oyaw, ov


def linear_mpc_control(xref, xbar, x0, dref):
    """
    linear mpc control 线性MPC控制

    xref: reference point 参考点
    xbar: operational point 操作点
    x0: initial state 初始状态
    dref: reference steer angle 参考转向角
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B, C = get_linear_model_matrix(xbar[2, t], xbar[3, t], dref[0, t])
        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= MAX_DSTEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        oa = get_nparray_from_matrix(u.value[0, :])
        odelta = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    """
    计算参考轨迹
    """
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref


def check_goal(state, goal, tind, nind):
    """
    检查全局终止条件
    """
    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)

    isgoal = d <= GOAL_DIS

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = abs(state.v) <= STOP_SPEED

    if isgoal and isstop:
        return True

    return False


def run(cx, cy, cyaw, ck, sp, dl, state):
    """
    主函数

    cx: course x position list 规划x坐标
    cy: course y position list 规划y坐标
    cyaw: course yaw position list 规划朝向
    ck: course curvature list 规划曲率
    sp: speed profile 规划速度
    dl: course tick [m] 样条线参数

    """

    goal = [cx[-1], cy[-1]]  # 设定目标点为最后一个航迹点

    target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)

    odelta, oa = None, None

    cyaw = smooth_yaw(cyaw)

    # 计算参考轨迹
    xref, target_ind, dref = calc_ref_trajectory(
        state, cx, cy, cyaw, ck, sp, dl, target_ind
    )
    # print(xref)

    x0 = [state.x, state.y, state.v, state.yaw]  # current state

    # MPC算法主函数
    oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
        xref, x0, dref, oa, odelta
    )

    # print([(x,y) for x, y in zip(ox, oy)])

    if odelta is not None:
        di, ai, vi = odelta[0], oa[0], ov[0]

    if check_goal(state, goal, target_ind, len(cx)):
        print("Goal")
        return Action()
    else:
        return Action(ai, di)


def calc_speed_profile(cx, cy, cyaw, target_speed):
    """
    计算速度
    """
    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = -target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def smooth_yaw(yaw):
    """
    朝向平滑化
    """
    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


def get_switch_back_course(dl):
    # 第一段航迹点
    ax = [0.0, 30.0, 6.0, 20.0, 35.0]
    ay = [0.0, 0.0, 20.0, 35.0, 20.0]
    # 第一段样条线
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)
    # 第二段航迹点
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]
    # 第二段样条线
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    # 连接两段
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)

    return cx, cy, cyaw, ck


def quat2yaw(w, x, y, z):
    r = Rotation.from_quat([x, y, z, w])
    # return np.math.atan(2 * ( w * z + x * y ) / (1- 2 * (x*x + y*y)))
    return np.squeeze(r.as_euler("zxy"))[1]


def pub(state_pub, action):
    ack = AckermannDriveStamped()
    ack.drive.speed = float(action.vel)
    ack.drive.steering_angle = float(action.steer)
    ack.drive.steering_angle_velocity = 1.0
    ack.drive.acceleration = float(action.acc)
    ack.drive.jerk = 1.0

    if state_pub is not None:
        state_pub.publish(ack)


def sub(msg):
    s = State()
    s.x = msg.pose.pose.position.x
    s.y = msg.pose.pose.position.y
    s.yaw = quat2yaw(
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
    )
    s.v = np.linalg.norm([msg.twist.twist.linear.x, msg.twist.twist.linear.y], ord=2)
    s.steer = np.math.atan(
        msg.twist.twist.angular.z * WB / s.v
    )  
    # cm angular speed to frontwheel steer

    # model predict and optimize control 预测将来状态
    action = run(cx, cy, cyaw, ck, sp, dl, s)
    action.vel = s.v + action.acc
    print("state", s.x, s.y, s.yaw, s.v, "action", action.acc, action.steer, action.vel)

    # publish cmd 发布命令
    pub(cmd_publisher, action)


if __name__ == "__main__":
    rospy.init_node("demo_nav_lmpc")
    rospy.Rate(ROSRATE)  # control rate 控制频率

    # Vehicle parameters 车辆参数
    # LENGTH = rospy.get_param("~veh_length", 4.5)  # [m] 长
    # WIDTH = rospy.get_param("~veh_width", 2.0)  # [m] 宽
    # BACKTOWHEEL = rospy.get_param("~veh_backtowheel", 1.0)  # [m] 后轮
    # WHEEL_LEN = rospy.get_param("~veh_wheel_length", 0.3)  # [m] 轮子长
    # WHEEL_WIDTH = rospy.get_param("~veh_wheel_width", 0.2)  # [m] 轮宽
    # TREAD = rospy.get_param("~veh_tread", 0.7)  # [m] 胎面
    WB = rospy.get_param("~veh_wb", 0.26)  # [m] 轴距

    MAX_STEER = rospy.get_param(
        "~max_steer", np.deg2rad(45.0)
    )  
    # maximum steering angle [rad] 最大转角
    MAX_DSTEER = rospy.get_param(
        "~max_dspeed", np.deg2rad(30.0)
    )  
    # maximum steering speed [rad/s] 最大角速度
    MAX_SPEED = rospy.get_param("~max_speed", 55.0 / 3.6)  
    # maximum speed [m/s] 最大速度
    MIN_SPEED = rospy.get_param("~min_speed", -20.0 / 3.6)  
    # minimum speed [m/s] 最大倒车速度
    MAX_ACCEL = rospy.get_param("~max_acc", 1.0)  
    # maximum accel [m/ss] 最大加速度

    # MPC parameters
    R = np.diag([0.01, 0.01])
    # rospy.get_param("~cost_matrix", np.diag([0.01, 0.01]))
    # input cost matrix 输入成本矩阵

    Rd = np.diag([0.01, 1.0])
    # rospy.get_param("~diff_cost_matrix", np.diag([0.01, 1.0]))
    # # input difference cost matrix 输入不同的状态

    Q = np.diag([1.0, 1.0, 0.5, 0.5])
    # rospy.get_param("~state_cost_matrix", np.diag([1.0, 1.0, 0.5, 0.5]))
    # # state cost matrix 状态成本矩阵

    Qf = Q  # state final matrix 状态矩阵

    cmd_publisher = rospy.Publisher(
        "/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=1
    )

    # prepare
    print("demo lmpc path tracking start!")

    dl = 0.1  # course tick 样条线参数
    cx, cy, cyaw, ck = get_switch_back_course(dl)  # 调用路径规划模块 输出x坐标 y坐标 朝向 曲率

    sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)  # 规划速度曲线

    # start 开始
    while not rospy.is_shutdown():
        # load current state 读取当前状态
        rospy.Subscriber("/odom", Odometry, sub, queue_size=1, buff_size=52428800)

        # loop 迭代
        rospy.spin()
