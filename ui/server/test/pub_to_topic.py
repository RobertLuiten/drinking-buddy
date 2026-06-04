import roslibpy
import json


ros = roslibpy.Ros(host='localhost', port=9090)
print("Connecting to ROS...")
try:
    ros.run(timeout=2)
except roslibpy.core.RosTimeoutError:
    print("Ros connection failed!")

if ros.is_connected:
    print("ROS-nnected!")
else:
    print("It's ROS-OVER")

robot_status_chan = roslibpy.Topic(ros, '/robot_status', 'std_msgs/String')
robot_map_chan = roslibpy.Topic(ros, '/robot_out_map', 'std_msgs/String')


test_robot_status = dict()
test_robot_status["state"] = "TEST"
test_robot_status["status"] = "TESTING TESTING READ ALL ABOUT IT!"
robot_status_chan.publish(roslibpy.Message({'data': json.dumps(test_robot_status)}))
print("Published robot status")