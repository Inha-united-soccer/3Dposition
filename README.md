1. best.pt (cone, ball) -> ball_detector_node_best -> rename (ball_detector_node)
2. yolo.pt (person, ball) -> ball_detector_node_yolo -> rename (ball_detector_node)


colcon build 
source install/setup.bash
ros2 launch perception soccer_bot.launch.py
