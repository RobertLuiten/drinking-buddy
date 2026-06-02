// Connect to rosbridge
console.log("Loading rosbridge!");
const ros = new ROSLIB.Ros({
    url: "ws://172.28.7.137:9090"
});

ros.on('connection', () => console.log("Connected to rosbridge"));
ros.on('error', err => console.log("Error:", err));
ros.on('close', () => console.log("Connection closed"));

// topic for bot to send requests to the robot
const robotRequest = new ROSLIB.Topic({
    ros: ros,
    name: "/robot_requests",
    messageType: "std_msgs/String"
});

// topic for robot to recieve status updates
const robotStatus = new ROSLIB.Topic({
    ros: ros,
    name: "/robot_status",
    messageType: "std_msgs/String"
});

// topic for robot to send & receive location data
const robotBroadcastLocation = new ROSLIB.Topic({
    ros: ros,
    name: "/robot_broadcast_location",
    messageType: "std_msgs/String"
});

// Request pose list on load
window.onload = function() {
  robotRequest.publish(new ROSLIB.Message({
    data: JSON.stringify({ command: "get_init_status" })
  }));
};



// Receive status updates
robotStatus.subscribe(function(msg) {
  const cur_status = JSON.parse(msg.data);
  display_status(cur_status);
});

function display_status(cur_status) {
  window.alert("display_status not implemented!");
}

// Receive robot location
robotBroadcastLocation.subscribe(function(msg) {
  const loc = JSON.parse(msg.data);
  display_status(cur_status);
});

function update_location(cur_status) {
  window.alert("update_location not implemented!");
}

function display_status(cur_status) {
  console.log(" ");
}

function clickNavigationMap(event) {
    var img_x = event.target.width;
    var img_y = event.target.height;
    var click_x = event.offsetX;
    var click_y = event.offsetY;
    var navigation_list = document.getElementById('navigation_list');
    var entry = document.createElement('li');
    
    robotBroadcastLocation.publish(new ROSLIB.Message({
      data: JSON.stringify({
        img_x : img_x,
        img_y : img_y,
        click_x : click_x,
        click_y : click_y
      })
    }));
    entry.appendChild(document.createTextNode('Sent Click (' + click_x + ',' + click_y + ') out of Image [' + img_x + ',' + img_y + '] to rosbridge'));
    navigation_list.appendChild(entry);
}