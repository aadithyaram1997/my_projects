import rclpy
from rclpy.node import Node

import sys, termios, tty

from nav_msgs.msg import Odometry

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def status_def(status):
    return (status + 1) % 15     


class TeleopClass(Node):
    def __init__(self, settings):
        super().__init__('teleop_twist_keyboard')

        self.timer = self.create_timer(0.01, self.mytimercallback)
        self.create_subscription(Odometry, "/odom", self.pose_cb, 10)
        self.settings = settings
        self.pose = 0
        self.poses = []

    def pose_cb(self, msg):
        self.pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

    def mytimercallback(self):
        key = getKey(self.settings)
        if key == 't':
            self._logger.info(str(self.pose))
        else:
            if key == '\x03':
                exit()

        restoreTerminalSettings(self.settings)  


def main(args=None):
    rclpy.init()
    teleop_twist_keyboard = TeleopClass(saveTerminalSettings())
    try:
        rclpy.spin(teleop_twist_keyboard)
    except KeyboardInterrupt:
        pass
    teleop_twist_keyboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
