import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymavlink import mavutil
import threading
import time

class MavlinkBridge(Node):
    def __init__(self):
        super().__init__('mavlink_bridge')
        self.subscription = self.create_subscription(
            Twist, '/cmd/evade', self.cmd_callback, 10)
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
        self.master.wait_heartbeat()
        self.get_logger().info('MAVLink heartbeat received')

    def cmd_callback(self, msg):
        self.get_logger().info(f'Command received: vx={msg.linear.x}')
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0,
            msg.linear.x, msg.linear.y, msg.linear.z,
            0, 0, 0, 0, 0)

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()