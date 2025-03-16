# 发布目标位置
rostopic pub /tianracer/move_base_simple/goal geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose: 
  position: 
    x: -0.005710601806640625
    y: 3.0816142559051514
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.7104248547260369
    w: 0.7037730641247144" --once

