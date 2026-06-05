from flask import Flask, send_from_directory, render_template, url_for
import roslibpy
from flask_socketio import SocketIO, emit
import json

## VAR INITIALIZATION
robot_status = dict()

## ROS SETUP
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
robot_status['connected'] = ros.is_connected

# NOTE: ALL ROS TOPICS HAVE THEIR DATA WITHIN THE DATA FIELD
# /robot_requests: will send commands *directly* within the data field (no json)
# /robot_status: will send a json string with "state" and "status" fields within the data field
# /robot_in_map: will send a json string with "img_x", "img_y", "click_x", and "click_y" fields within the data field
# /robot_out_map: will send a json string with "map_x", "map_y", "robot_x", and "robot_y" fields within the data field
# 
# The only thing that should be coming out of /robot_requests is the string command "start"
#
# in /robot_status, "state" and "status" fields are ill-defined. 
# Currently though, "state" is where the string that describes the robot's current state (e.g. "idle", "moving", "error") should go,
# and "status" is where any additional information about the robot's current state should go 
# (e.g. if state is "error", status might be "stuck on obstacle")
# 
# Feel free to add more fields to /robot_status, just tell Derick so he can add it to the frontend
robot_request_broadcast = roslibpy.Topic(ros, '/robot_requests', 'std_msgs/String')
robot_status_listener = roslibpy.Topic(ros, '/robot_status', 'std_msgs/String')
robot_map_broadcast = roslibpy.Topic(ros, '/robot_in_map', 'std_msgs/String')
robot_map_listener = roslibpy.Topic(ros, '/robot_out_map', 'std_msgs/String')

# APP/SOCKET INITIALIZATION

app = Flask(__name__)  
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)


## WEB ROUTING
@app.route('/')  
def root():  
    return render_template('main.html')

## ROS ROUTING
robot_status_listener.subscribe(lambda msg: handle_new_status(msg['data']))
robot_map_listener.subscribe(lambda msg: handle_new_map(msg['data']))

## SOCKET ROUTING
@socketio.on('connect')
def client_connect(auth):
    global robot_status
    emit('status_update', json.dumps(robot_status))

@socketio.on('robot_command')
def robot_command(cmd):
    print("client requested " + cmd)
    robot_request_broadcast.publish(roslibpy.Message({'data': cmd}))
    emit('command_ack', "Done!")

@socketio.on('map_click')
def send_new_click(map):
    print(map)
    robot_map_broadcast.publish(roslibpy.Message({'data': json.dumps(map)}))
    # test_broadcast_dot(map)

## HELPER FUNCTIONS

def handle_new_status(status_json):
    new_status = json.loads(status_json)
    global robot_status
    robot_status["state"] = new_status["state"]
    robot_status["status"] = new_status["status"]
    robot_status["connected"] = ros.is_connected
    socketio.emit('status_update', json.dumps(robot_status))


def handle_new_map(map_json):
    map = json.loads(map_json)
    out_map = dict()
    out_map["map_x"] = map["map_x"]
    out_map["map_y"] = map["map_y"]
    out_map["robot_x"] = map["robot_x"]
    out_map["robot_y"] = map["robot_y"]
    socketio.emit('map_info', json.dumps(out_map))

## TESTING FUNCTION

def test_broadcast_dot(map) :
    out_map = dict()
    out_map["map_x"] = map["img_x"]
    out_map["map_y"] = map["img_y"]
    out_map["robot_x"] = map["click_x"] / 2
    out_map["robot_y"] = map["click_y"] / 2
    socketio.emit('map_info', json.dumps(out_map))

# Main
if __name__ == '__main__':
    socketio.run(app)