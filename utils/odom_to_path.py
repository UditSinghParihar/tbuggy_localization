# odom_to_path.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        self.path = Path()
        self.pub = self.create_publisher(Path, '/tbuggy/path', 10)
        self.sub = self.create_subscription(Odometry, '/tbuggy/odom', self.cb, 10)

    def cb(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.header = msg.header
        self.path.poses.append(pose)
        self.pub.publish(self.path)

rclpy.init()
rclpy.spin(OdomToPath())
