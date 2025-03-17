# 发布第一个点 (1.0, 0.0, 0.0)
rostopic pub /clicked_point geometry_msgs/PointStamped "{header: {stamp: now, frame_id: 'world'}, point: {x: 1.0, y: 0.0, z: 0.0}}" -1

# 发布第二个点 (2.0, 1.0, 0.0)
rostopic pub /clicked_point geometry_msgs/PointStamped "{header: {stamp: now, frame_id: 'world'}, point: {x: 2.0, y: 1.0, z: 0.0}}" -1

# 发布第三个点 (3.0, 0.0, 0.0)
rostopic pub /clicked_point geometry_msgs/PointStamped "{header: {stamp: now, frame_id: 'world'}, point: {x: 3.0, y: 0.0, z: 0.0}}" -1

# 发布第四个点 (2.0, -1.0, 0.0)
rostopic pub /clicked_point geometry_msgs/PointStamped "{header: {stamp: now, frame_id: 'world'}, point: {x: 2.0, y: -1.0, z: 0.0}}" -1

# 发布第五个点，回到起点 (1.0, 0.0, 0.0)
rostopic pub /clicked_point geometry_msgs/PointStamped "{header: {stamp: now, frame_id: 'world'}, point: {x: 1.0, y: 0.0, z: 0.0}}" -1