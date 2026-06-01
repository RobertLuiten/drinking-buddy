// Connect to rosbridge
console.log("Loading rosbridge!");
const ros = new ROSLIB.Ros({
    url: "ws://172.28.7.137:9090"
});

ros.on('connection', () => console.log("Connected to rosbridge"));
ros.on('error', err => console.log("Error:", err));
ros.on('close', () => console.log("Connection closed"));

// Publisher for pose manager commands
const poseMgrPub = new ROSLIB.Topic({
    ros: ros,
    name: "/pose_mgr",
    messageType: "std_msgs/String"
});

// Subscriber for pose list updates
const poseListSub = new ROSLIB.Topic({
    ros: ros,
    name: "/pose_list",
    messageType: "std_msgs/msg/String"
});

// Request pose list on load
window.onload = function() {
  poseMgrPub.publish(new ROSLIB.Message({
    data: JSON.stringify({ command: "list" })
  }));
};

// Save pose
function savePose() {
  const name = document.getElementById("poseName").value;
  if (!name) return;

  poseMgrPub.publish(new ROSLIB.Message({
    data: JSON.stringify({
      command: "save",
      name: name
    })
  }));
}

// Restore pose
function restorePose(name) {
  poseMgrPub.publish(new ROSLIB.Message({
    data: JSON.stringify({
      command: "restore",
      name: name
    })
  }));
}

// Receive pose list
poseListSub.subscribe(function(msg) {
  const names = JSON.parse(msg.data);
  renderPoseCards(names);
});

// Render pose cards
function renderPoseCards(poses) {
    const container = document.getElementById("poseList");
    container.innerHTML = "";

    poses.forEach(name => {
    if (!name) return;
    const card = document.createElement("div");
    card.className = "pose-card";
    card.innerText = name;
    card.onclick = () => restorePose(name);
    container.appendChild(card);
    });
}