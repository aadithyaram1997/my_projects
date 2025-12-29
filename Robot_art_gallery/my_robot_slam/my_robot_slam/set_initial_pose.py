#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

def main():
    global mypub
    rclpy.init()
    myfirstpublisher = rclpy.create_node('publisher')
    mypub = myfirstpublisher.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    myfirstpublisher.create_timer(0.1, mytimercallback)
    try:
        rclpy.spin_once(myfirstpublisher)
    except KeyboardInterrupt:
        pass

    myfirstpublisher.destroy_node()
    rclpy.shutdown()

def mytimercallback():
    global mypub
    mymsg = PoseWithCovarianceStamped()
    #mymsg.header.stamp = publisher.get_clock().now().to_msg()
    mymsg.header.frame_id = 'map'
    mymsg.pose.pose.position.x = -6.402865235847956e-05
    mymsg.pose.pose.position.y = 0.002986915702010021
    mymsg.pose.pose.position.z = 0.007236296735348522
    mymsg.pose.pose.orientation.x = -0.0029668523454207747
    mymsg.pose.pose.orientation.y = 0.002880361435779419
    mymsg.pose.pose.orientation.z = 0.7098823588795448
    mymsg.pose.pose.orientation.w = 0.7043081270699445
    mypub.publish(mymsg)
    exit()

if __name__ == '__main__':
    main()
