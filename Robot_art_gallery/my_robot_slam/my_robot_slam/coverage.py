#!/usr/bin/env python3

import csv
import os
import cv2
import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python import get_package_share_directory
from cv_bridge import CvBridge

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

from my_robot_slam.detection import YOLOX


class MinimalActionClient(Node):

    def __init__(self):
        super().__init__('minimal_action_client')
        self._client = ActionClient(self, FollowWaypoints, '/FollowWaypoints')
        self.create_subscription(Image,"/camera/image_raw",self.imageflow_callback, 10)
        self.bridge = CvBridge()
        self.image = None
        self.waypoint_nr = 0

        self.image_list = []
        self.image_dict = {}

    def imageflow_callback(self,msg:Image) -> None:
        self.image = self.bridge.imgmsg_to_cv2(msg,"bgr8")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.send_points(mgoal)
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        
        if self.waypoint_nr != feedback.current_waypoint:
            self.waypoint_nr = feedback.current_waypoint
            self.get_logger().info('Took Pic at point ' + str(self.waypoint_nr))
            dir = get_package_share_directory('my_robot_slam') + '/pics/'
            if not os.path.exists(dir):
                os.mkdir(dir)
            cv2.imwrite(dir + str(self.waypoint_nr) + '.jpg', self.image)
            self.image_list.append(self.image)
            if self.waypoint_nr == 44:
                self.yolox = YOLOX()
                result_img = np.zeros([1080,1,3],dtype=np.uint8)
                result_img[:] = 255
                for i, img_detect in enumerate(self.image_list):
                    self.get_logger().info('Scanning: ' + str(i + 1) + '/' + str(len(self.image_list)))                    
                    try:
                        self.image_dict, result_img = \
                            self.yolox.image_detection(img_detect, self.image_dict, result_img)
                    except:
                        pass
                scale_percent = 1080 / result_img.shape[1]
                width = int(result_img.shape[1] * scale_percent)
                height = int(result_img.shape[0] * scale_percent)
                dim = (width, height)
                resized = cv2.resize(result_img, dim, interpolation = cv2.INTER_AREA)

                font                   = cv2.FONT_HERSHEY_SIMPLEX
                bottomLeftCornerOfText = (10,25)
                fontScale              = 1
                fontColor              = (255,255,255)
                thickness              = 2
                lineType               = 2

                resized = cv2.putText(resized,str(self.image_dict), 
                    bottomLeftCornerOfText,
                    font,
                    fontScale,
                    fontColor,
                    thickness,
                    lineType)

                cv2.imshow('paintings', resized)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.missed_waypoints))
        # Shutdown after receiving a result
        rclpy.shutdown()

    def send_points(self, points):
        # self.get_logger().info('Waiting for action server...')
        
        msg = FollowWaypoints.Goal()
        msg.poses = points
        
        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)    
        self.get_logger().info('Sending goal request...')


def main(args=None):
    global mgoal
    rclpy.init(args=args)
    action_client = MinimalActionClient()

    mgoal = []
    with open(get_package_share_directory('my_robot_slam') + '/resource/poses.csv', newline='') as f:
        reader = csv.reader(f)
        poses = list(reader)

    for pose in poses:
        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"
        waypoint.pose.position.x =  float(pose[0])
        waypoint.pose.position.y =  float(pose[1])
        waypoint.pose.position.z =  float(pose[2])

        waypoint.pose.orientation.x = float(pose[3])
        waypoint.pose.orientation.y = float(pose[4])
        waypoint.pose.orientation.z = float(pose[5])
        waypoint.pose.orientation.w = float(pose[6])

        mgoal.append(waypoint)

    action_client.send_points(mgoal)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
