#!/usr/bin/env python3
"""

Path tracking simulation with iterative linear model predictive control for speed and steer control
速度和转向控制的迭代线性模型预测控制算法及路径跟踪仿真

author: Atsushi Sakai (@Atsushi_twi)
revised: Yun Yuan (@uwmyuan)
"""
from ast import Return
from re import A
from statistics import variance
import cvxpy  # 凸优化库，用于求解MPC问题
import math
import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped  # 阿克曼转向消息类型
from nav_msgs.msg import Odometry  # 里程计消息类型
from scipy.spatial.transform import Rotation  # 用于四元数到欧拉角的转换
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

try:
    import os
    import sys
    # 添加当前目录到Python路径
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    import cubic_spline_planner  # 这里以三次样条曲线路径规划器为例，也可以更换其他规划算法
except Exception as e:
    rospy.logerr("导入cubic_spline_planner失败: %s", str(e))
    raise

NX = 4  # x = x, y, v, yaw 状态空间维度 包括x轴 y轴 线速度 偏航（朝向）
NU = 2  # a = [accel, steer]  控制空间维度 包括加速度和转向角速度
T = 5   # horizon length 预测时间窗宽度，即预测未来5个时间步
DT = 0.1  # [s] time tick 时钟单位，每个时间步为0.1秒

#  mpc parameters MPC参数设置
GOAL_DIS = 1.  # goal distance 与目标点的距离阈值，单位：米
STOP_SPEED = 0.3 / 3.6  # stop speed 停止速度，除以3.6单位转化为m/s

# 迭代终止条件参数
MAX_ITER = 5  # Max iteration 最大迭代次数，控制计算量
DU_TH = 0.05  # iteration finish param 连续两次结果之差的阈值，用于判断迭代是否收敛

TARGET_SPEED = 0.3  # [m/s] target speed 巡航速度，单位m/s
N_IND_SEARCH = 10  # Search index number 搜索最近航迹点的范围，提高效率

ROSRATE = 10  # ROS节点的运行频率，单位Hz

# 添加全局变量来存储路径和目标
global_path = None
global_goal = None


# 初始化全局变量
cx = cy = cyaw = ck = sp = cmd_publisher = local_path_publisher = None
dl = 0.2  # 路径采样间隔

# Get vehicle parameters from ROS parameter server
# 从ROS参数服务器获取车辆参数
param_ns = "~"  # Use node private namespace / 使用节点私有命名空间
WB = rospy.get_param(param_ns + "veh_wb", 0.26)  # Wheelbase [m] / 轴距
MAX_STEER = rospy.get_param(param_ns + "max_steer", np.deg2rad(15.0))  # Max steering angle [rad] / 最大转向角
MAX_DSTEER = rospy.get_param(param_ns + "max_dspeed", np.deg2rad(30.0))  # Max steering rate [rad/s] / 最大转向角速度
MAX_SPEED = rospy.get_param(param_ns + "max_speed", 15.0 / 3.6)  # Max speed [m/s] / 最大速度
MIN_SPEED = rospy.get_param(param_ns + "min_speed", -10.0 / 3.6)  # Min speed [m/s] / 最小速度
MAX_ACCEL = rospy.get_param(param_ns + "max_acc", 1.0)  # Max acceleration [m/s²] / 最大加速度

# MPC cost matrices
# MPC代价矩阵
R = np.diag([0.01, 0.01])  # Input cost matrix / 输入成本矩阵
Rd = np.diag([0.01, 1.0])  # Input difference cost matrix / 输入差分成本矩阵
Q = np.diag([2.0, 2.0, 0.3, 0.5])  # State cost matrix / 状态成本矩阵
Qf = Q * 2  # Terminal state cost matrix / 终端状态成本矩阵       

class State:
    """
    vehicle state class
    车辆状态类，用于存储车辆的位置、朝向、速度等状态信息
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, s=0.0):
        self.x = x        # x坐标，单位：米
        self.y = y        # y坐标，单位：米
        self.yaw = yaw    # 偏航角（朝向），单位：弧度
        self.v = v        # 线速度，单位：米/秒
        self.steer = s    # 转向角，单位：弧度
        self.predelta = None  # 上一时刻的转向角，用于平滑控制


class Action:
    """
    vehicle action class
    车辆行为类，用于存储控制指令
    """

    def __init__(self, acc=0.0, steer=0.0, vel=0.0):
        self.acc = acc      # 加速度，单位：米/秒²
        self.vel = vel      # 速度，单位：米/秒
        self.steer = steer  # 转向角，单位：弧度


def pi_2_pi(angle):
    """
    rescale angle within a range [-pi, pi]
    将角度归一化到[-pi, pi]范围内，避免角度累积误差
    """
    while angle > math.pi:
        angle = angle - 2.0 * math.pi

    while angle < -math.pi:
        angle = angle + 2.0 * math.pi

    return angle


def get_linear_model_matrix(v, phi, delta):
    """
    Matrix form of Ackermann vehicle dynamics model
    阿克曼车辆动力学模型的矩阵形式，用于MPC预测
    
    参数：
    v: 当前速度
    phi: 当前偏航角
    delta: 当前转向角
    
    返回：
    A, B, C: 线性化后的系统矩阵
    """
    # 状态转移矩阵A
    A = np.zeros((NX, NX))
    A[0, 0] = 1.0  # x坐标的自传递
    A[1, 1] = 1.0  # y坐标的自传递
    A[2, 2] = 1.0  # 速度的自传递
    A[3, 3] = 1.0  # 偏航角的自传递
    A[0, 2] = DT * math.cos(phi)  # 速度对x坐标的影响
    A[0, 3] = -DT * v * math.sin(phi)  # 偏航角对x坐标的影响
    A[1, 2] = DT * math.sin(phi)  # 速度对y坐标的影响
    A[1, 3] = DT * v * math.cos(phi)  # 偏航角对y坐标的影响
    A[3, 2] = DT * math.tan(delta) / WB  # 速度对偏航角的影响（与轴距和转向角有关）

    # 控制输入矩阵B
    B = np.zeros((NX, NU))
    B[2, 0] = DT  # 加速度对速度的影响
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)  # 转向角速度对偏航角的影响

    # 非线性项补偿矩阵C
    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi  # x坐标的非线性补偿
    C[1] = -DT * v * math.cos(phi) * phi  # y坐标的非线性补偿
    C[3] = -DT * v * delta / (WB * math.cos(delta) ** 2)  # 偏航角的非线性补偿

    return A, B, C


def update_state(state, a, delta):
    """
    Update state using the Ackermann model
    使用阿克曼模型更新车辆状态
    
    参数：
    state: 当前状态
    a: 加速度
    delta: 转向角
    
    返回：
    更新后的状态
    """

    # 输入限制检查
    if delta >= MAX_STEER:
        delta = MAX_STEER  # 限制最大转向角
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER  # 限制最小转向角

    # 使用运动学模型更新状态
    state.x = state.x + state.v * math.cos(state.yaw) * DT  # 更新x坐标
    state.y = state.y + state.v * math.sin(state.yaw) * DT  # 更新y坐标
    state.yaw = state.yaw + state.v / WB * math.tan(delta) * DT  # 更新偏航角
    state.v = state.v + a * DT  # 更新速度

    # 速度限制
    if state.v > MAX_SPEED:
        state.v = MAX_SPEED  # 限制最大速度
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED  # 限制最小速度（倒车速度）

    return state


def get_nparray_from_matrix(x):
    """
    a short rename from matrix flatten
    将矩阵转换为一维数组，便于后续处理
    """
    return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind):
    """
    find the index of the nearest waypoint
    找到最近的航迹点序号，用于确定当前跟踪的目标点
    
    参数：
    state: 当前车辆状态
    cx, cy: 路径x,y坐标列表
    cyaw: 路径朝向列表
    pind: 上一次的索引位置（用于加速搜索）
    
    返回：
    ind: 最近点索引
    mind: 到最近点的距离（带符号）
    """

    # 计算当前位置到前N_IND_SEARCH个路径点的距离
    dx = [state.x - icx for icx in cx[pind : (pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind : (pind + N_IND_SEARCH)]]

    # 计算欧氏距离的平方
    d = [idx**2 + idy**2 for (idx, idy) in zip(dx, dy)]

    # 找到最小距离及其索引
    mind = min(d)
    ind = d.index(mind) + pind
    mind = math.sqrt(mind)  # 转换为实际距离

    # 计算最近点相对于车辆的方向
    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y
    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    
    # 根据角度确定距离符号（用于判断车辆是否超过了路径点）
    if angle < 0:
        mind *= -1

    return ind, mind


def predict_motion(x0, oa, od, xref):
    """
    Calculate vehicle states within the current time window
    计算当前时间窗内车辆状态的预测轨迹
    
    参数：
    x0: 初始状态
    oa: 加速度控制序列
    od: 转向角控制序列
    xref: 参考轨迹
    
    返回：
    xbar: 预测的状态轨迹
    """
    xbar = xref * 0.0  # 初始化预测轨迹矩阵
    
    # 设置初始状态
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    # 使用车辆模型逐步预测未来状态
    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (ai, di, i) in zip(oa, od, range(1, T + 1)):
        state = update_state(state, ai, di)  # 使用控制输入更新状态
        xbar[0, i] = state.x  # 存储预测的x坐标
        xbar[1, i] = state.y  # 存储预测的y坐标
        xbar[2, i] = state.v  # 存储预测的速度
        xbar[3, i] = state.yaw  # 存储预测的偏航角

    return xbar


def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    Iteratively update operation points for MPC control
    迭代更新MPC控制的操作点，提高非线性系统的控制精度
    
    参数：
    xref: 参考轨迹
    x0: 初始状态
    dref: 参考转向角
    oa, od: 上一时刻的控制序列
    
    返回：
    oa, od: 优化后的控制序列
    ox, oy, oyaw, ov: 预测的状态轨迹
    """

    # 如果没有上一时刻的控制序列，初始化为零
    if oa is None or od is None:
        oa = [0.0] * T
        od = [0.0] * T

    # 迭代求解MPC问题
    for _ in range(MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref)  # 预测轨迹
        poa, pod = oa[:], od[:]  # 保存上一次的控制序列
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)  # 求解线性MPC
        
        # 计算控制变化量，判断是否收敛
        du = sum(abs(oa - poa)) + sum(abs(od - pod))  
        if du <= DU_TH:  # 如果变化量小于阈值，认为已收敛
            break
    else:
        print("迭代达到最大次数")

    return oa, od, ox, oy, oyaw, ov


def linear_mpc_control(xref, xbar, x0, dref):
    """
    Linear MPC control
    线性MPC控制，求解优化问题得到最优控制序列
    
    参数：
    xref: 参考轨迹 (reference point)
    xbar: 操作点，用于线性化 (operational point)
    x0: 初始状态 (initial state)
    dref: 参考转向角 (reference steering angle)
    
    返回：
    oa, odelta: 优化后的加速度和转向角控制序列
    ox, oy, oyaw, ov: 预测的状态轨迹
    """

    # 定义优化变量
    x = cvxpy.Variable((NX, T + 1))  # 状态变量
    u = cvxpy.Variable((NU, T))      # 控制变量

    # 初始化代价函数和约束条件
    cost = 0.0
    constraints = []

    # 构建MPC优化问题
    for t in range(T):
        # 控制输入代价
        cost += cvxpy.quad_form(u[:, t], R)  #

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
    prob.solve(solver=cvxpy.OSQP, verbose=False)

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
    Calculate reference trajectory
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
    Check global termination conditions
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
    Main function
    主函数

    cx: course x position list 规划x坐标 (planned x coordinates)
    cy: course y position list 规划y坐标 (planned y coordinates)
    cyaw: course yaw position list 规划朝向 (planned orientations)
    ck: course curvature list 规划曲率 (planned curvatures)
    sp: speed profile 规划速度 (planned speeds)
    dl: course tick [m] 样条线参数 (spline parameter)

    """

    goal = [cx[-1], cy[-1]]  # 设定目标点为最后一个航迹点
                             # Set the goal point as the last trajectory point

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

    # 发布局部路径
    if ox is not None and oy is not None and local_path_publisher is not None:
        publish_local_path(ox, oy, oyaw)

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
    Calculate speed profile
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
    Smooth yaw angle list
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


# def get_switch_back_course(dl):
#     """
#     Generate a switch-back course for testing
#     生成一个回环测试路径
#     """
#     # First segment waypoints
#     # 第一段航迹点 
#     ax = [0.0, 30.0, 6.0, 20.0, 35.0]
#     ay = [0.0, 0.0, 20.0, 35.0, 20.0]
#     # First segment spline
#     # 第一段样条线
#     cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)
#     # Second segment waypoints
#     # 第二段航迹点
#     ax = [35.0, 10.0, 0.0, 0.0]
#     ay = [20.0, 30.0, 5.0, 0.0]
#     # Second segment spline
#     # 第二段样条线
#     cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)
#     cyaw2 = [i - math.pi for i in cyaw2]
#     # Connect two segments
#     # 连接两段
#     cx.extend(cx2)
#     cy.extend(cy2)
#     cyaw.extend(cyaw2)
#     ck.extend(ck2)

#     return cx, cy, cyaw, ck


def quat2yaw(w, x, y, z):
    """
    Convert quaternion to yaw angle
    四元数转换为偏航角
    """
    r = Rotation.from_quat([x, y, z, w])
    return r.as_euler("xyz")[2]



# 在主函数中获取参数
base_frame_id = rospy.get_param("~base_frame_id", "base_footprint")

# 在 pub 函数中使用
def pub(state_pub, action):
    """
    Publish control commands to ROS topic
    发布控制命令到ROS话题
    """

     # 限制速度变化率，防止突然加速或减速
    if abs(action.vel) > 1.0:
        action.vel = 1.0 if action.vel > 0 else -1.0
        
    # 限制转向角变化率，防止突然转向
    if abs(action.steer) > MAX_STEER * 0.8:
        action.steer = MAX_STEER * 0.8 if action.steer > 0 else -MAX_STEER * 0.8

    ack = AckermannDriveStamped()
    ack.header.stamp = rospy.Time.now()
    ack.header.frame_id = base_frame_id  # 使用从参数获取的帧ID
    ack.drive.speed = float(action.vel)
    ack.drive.steering_angle = float(action.steer)
    # ack.drive.steering_angle_velocity = 1.0
    ack.drive.acceleration = float(action.acc)
    # ack.drive.jerk = 1.0

    if state_pub is not None:
        state_pub.publish(ack)


def sub(msg):
    """
    Callback function for odometry messages
    Process vehicle state and generate control commands
    里程计消息的回调函数，处理车辆状态并生成控制命令
    """
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

    # Convert center of mass angular speed to front wheel steering angle
    # cm angular speed to frontwheel steer 
    # Prevent division by zero error
    # 将质心角速度转换为前轮转向角，防止除零错误
    if abs(s.v) > 0.001:
        s.steer = np.math.atan(msg.twist.twist.angular.z * WB / s.v)
    else:
        s.steer = 0.0

    if cx is not None:
        # Predict future states and optimize control using MPC
        # 使用MPC预测未来状态并优化控制
        action = run(cx, cy, cyaw, ck, sp, dl, s)
        action.vel = s.v + action.acc
        
        # Log current state and control actions for debugging
        rospy.loginfo("State: x=%.2f, y=%.2f, yaw=%.2f, v=%.2f | Action: acc=%.2f, steer=%.2f, vel=%.2f", 
                     s.x, s.y, s.yaw, s.v, action.acc, action.steer, action.vel)

        # Publish command to vehicle
        # 发布控制命令到车辆
        pub(cmd_publisher, action)
    else:
        # rospy.loginfo(f"sub cx {cx}")
        rospy.logwarn("Path not available yet, waiting for path message / 路径尚未可用，等待路径消息")
        return

def path_callback(msg):
    """
    Receive global path from navigation stack
    接收全局路径
    """

    
    if len(msg.poses) < 2:
        rospy.logwarn("Received path has insufficient points (less than 2) / 接收到的路径点数量不足")
        return
    
    # Extract waypoints from path message
    # 提取路径点
    x_list = []
    y_list = []
    yaw_list = []
    
    for pose in msg.poses:
        x_list.append(pose.pose.position.x)
        y_list.append(pose.pose.position.y)
        yaw_list.append(quat2yaw(
            pose.pose.orientation.w,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
    ))
    
    global cx, cy, cyaw, ck, sp, global_path
    # Smooth path using cubic spline
    # 使用样条曲线平滑路径
    cx, cy, cyaw, ck, _ = cubic_spline_planner.calc_spline_course(x_list, y_list, ds=dl)

    # Calculate speed profile along the path
    # 计算速度曲线
    sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)

    rospy.loginfo("smooth path created by calc_speed_profile")
    # rospy.loginfo(f"path cx {cx}")
    
    global_path = msg
    rospy.loginfo("Received new global path with %d points / 已接收新的全局路径，共 %d 个点", len(msg.poses), len(msg.poses))

def goal_callback(msg):
    """
    Receive global goal from navigation stack
    接收全局目标点
    """
    global global_goal
    global_goal = msg
    rospy.loginfo("Received new goal: x=%.2f, y=%.2f / 已接收新的目标点: x=%.2f, y=%.2f", 
                 msg.pose.position.x, msg.pose.position.y, 
                 msg.pose.position.x, msg.pose.position.y)


# 添加发布局部路径的函数
def publish_local_path(ox, oy, oyaw):
    """
    Publish local path from MPC prediction
    发布MPC预测的局部路径
    """
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "map"  # 使用地图坐标系
    
    for i in range(len(ox)):
        pose = PoseStamped()
        pose.header.stamp = path.header.stamp
        pose.header.frame_id = path.header.frame_id
        pose.pose.position.x = ox[i]
        pose.pose.position.y = oy[i]
        pose.pose.position.z = 0.0
        
        # 从偏航角计算四元数
        quat = Rotation.from_euler('z', oyaw[i]).as_quat()
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        path.poses.append(pose)
    
    local_path_publisher.publish(path)


if __name__ == "__main__":
    try:
        rospy.init_node("mpc_controller")  # Initialize ROS node / 初始化ROS节点
        rate = rospy.Rate(ROSRATE)  # Control frequency / 控制频率

        # Get namespace parameters
        # 获取命名空间参数
        robot_name = rospy.get_param('~robot_name', "tianracer")
        
        # Get topic parameters
        # 获取话题参数
        cmd_topic = rospy.get_param("~cmd_topic", "ackermann_cmd_stamped")
        
        # Create publishers and subscribers
        # 创建发布器和订阅器
        cmd_publisher = rospy.Publisher(
            cmd_topic, AckermannDriveStamped, queue_size=1
        )  # Control command publisher / 控制命令发布器
        
        # 添加局部路径发布器
        local_path_publisher = rospy.Publisher(
            "move_base/MPCLocalPlannerROS/local_plan", Path, queue_size=1
        )  # Local path publisher / 局部路径发布器

        # Subscribe to global path and goal
        # 订阅全局路径和目标
        path_sub = rospy.Subscriber("path", Path, path_callback, queue_size=1)
        goal_sub = rospy.Subscriber("goal", PoseStamped, goal_callback, queue_size=1)
        
        # Subscribe to odometry
        # 订阅里程计消息
        odom_sub = rospy.Subscriber("odom", Odometry, sub, queue_size=1)

        # Startup message
        # 启动消息
        rospy.loginfo("MPC controller started! / MPC 控制器已启动!")
        rospy.loginfo("Waiting for path and odometry messages... / 等待路径和里程计消息...")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("MPC controller terminated / MPC 控制器已终止")
    except Exception as e:
        rospy.logerr("MPC controller error: %s / MPC 控制器发生错误: %s", str(e), str(e))
