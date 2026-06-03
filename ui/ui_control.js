// Automatically calculate image size
const img = new Image();
img.src = 'placeholder.jpg'
var map_img = document.getElementById("navigation_map");
map_img.style = "background: url('" + img.src + " ');" +
  "background-size:" + img.naturalWidth / 2 + "px;" +  
  "width:" + img.naturalWidth / 2 + "px;" +  
  "height:" + img.naturalHeight / 2+ "px; object-fit: contain;"


// Connect to rosbridge
console.log("Loading rosbridge!");
const ros = new ROSLIB.Ros({
    url: "ws://172.28.7.144:9090"
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
  if (cur_status == null) {
    console.log("Empty Status");
    return;
  }
  console.log("Current Robot Status:");
  console.log("State: " + cur_status.state);
  console.log("Status: " + cur_status.status);
  document.getElementById("robotDetect").innerText = "Robot: DETECTED";
  document.getElementById("currentAction").innerText = "Current Action: " + cur_status.status;
}

// Receive robot location
robotBroadcastLocation.subscribe(function(msg) {
  const loc = JSON.parse(msg.data);
  display_status(cur_status);
});

function update_location(cur_status) {
  window.alert("update_location not implemented!");
}

function clickNavigationMap(event) {
    var click_dot = document.getElementById('click_dot');
    if (event.target == click_dot) {
      return;
    }
    var img_x = event.target.clientWidth;
    var img_y = event.target.clientHeight;
    var click_x = event.offsetX;
    var click_y = event.offsetY;
    var navigation_list = document.getElementById('navigation_list');
    var entry = document.createElement('li');
    
    click_dot.style = "height: 14px; width: 14px; margin-top: " + (click_y - (click_dot.height / 2)) + "px; margin-left: " + (click_x - (click_dot.width / 2)) + "px;";

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


function startRobot(event) {
  var start_button = document.getElementById("robotResetButton");
  var reset_button = document.getElementById("robotStartButton");
  start_button.disabled = false;
  reset_button.disabled = true;
  robotRequest.publish(new ROSLIB.Message({
    data: JSON.stringify({ command: "start" })
  }));
}
function resetPosition(event) {
  var start_button = document.getElementById("robotResetButton");
  var reset_button = document.getElementById("robotStartButton");
  start_button.disabled = true;
  reset_button.disabled = false;
  robotRequest.publish(new ROSLIB.Message({
    data: JSON.stringify({ command: "reset" })
  }));

}

function startVoice(event) {
  window.alert("voice control not implemented!");
}