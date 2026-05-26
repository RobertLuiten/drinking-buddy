# Extend the arm by +2 cm
import time
from std_msgs.msg import Float64

# Publish to Stretch's arm extension controller
arm_pub = self.create_publisher(Float64, '/stretch/arm/command', 10)

# Extend by +0.02 meters
cmd = Float64()
cmd.data = self.current_arm_position + 0.02
arm_pub.publish(cmd)

# Wait for motion to complete
time.sleep(1.0)

# Now print the TF again
t = self.tf_buffer.lookup_transform(
    'link_grasp_center',
    'supps',
    rclpy.time.Time(),
    rclpy.duration.Duration(seconds=0.5)
)

print("After extending arm 2cm:")
print("x =", t.transform.translation.x)
print("y =", t.transform.translation.y)
print("z =", t.transform.translation.z)
