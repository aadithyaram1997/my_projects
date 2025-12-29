#!/usr/bin/env python
import sys
from time import sleep

from numpy.core.numeric import NaN

#ros2
from sensor_msgs.msg import Image , LaserScan
import rclpy, math, scipy
from rclpy.node import Node
from px4_msgs.msg import Timesync, TrajectorySetpoint, VehicleCommand, OffboardControlMode, VehicleLocalPositionSetpoint, VehicleStatus, VehicleOdometry
import cv2
import numpy as np
from cv_bridge import CvBridge

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.c = 0
        self.yaw = 0.14     
        self.received_x = 0.0
        self.received_x_last = 0.0
        self.received_y = 0.0
        self.received_y_last = 0.0
        self.received_z = 0.0              
        self.movementSpeedFactor = 3.0
        self.hoverHeight = -1.5  #1.5
        self.upHeight = -6.0
        self.land_at_point = False
        self.notFound = True        
        self.no_color_list = 0
        self.getLastPos = False
        self.disableHover = False
        self.step = 10
        self.origin = [0.0,0.0,0.0]      
        self.reached_Altitude = False
        self.takeoff_success = False
        self.received_timestamp_sample = 0.0
        self.color_number = 0
        self.timestamp = 0
        self.control_mode_publisher = self.create_publisher(OffboardControlMode, '/OffboardControlMode_PubSubTopic', 10)
        self.trajectory_setpoint_publisher = self. create_publisher(TrajectorySetpoint, '/TrajectorySetpoint_PubSubTopic', 10)
        self.vehicle_setpoint_publisher = self. create_publisher(VehicleLocalPositionSetpoint, '/VehicleLocalPositionSetpoint_PubSubTopic', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/VehicleCommand_PubSubTopic', 10)
        self.timesync_sub = self.create_subscription(Timesync,'/Timesync_PubSubTopic', self.sub_callback,10)
        self.vehicleStatus_sub = self.create_subscription(VehicleStatus,'/VehicleStatus_PubSubTopic', self.vehicleStatus,10)
        self.odometry_sub = self.create_subscription(VehicleOdometry,'/VehicleOdometry_PubSubTopic', self.odometry,10)
        self.pub = self.create_publisher(Image, "processed", 10)
        self.create_subscription(Image, "/camera/image_raw", self.img_cb, 10)
        self.lidar_read = self.create_subscription(LaserScan, '/lidar/scan', self.lidar_read_callBack, 10)
        self.run()
        self.timer = self.create_timer(0.1, self.run)
      
    def lidar_read_callBack(self,msg):
        tempVal = np.array_split(msg.ranges, 4)      
        lidar_right = np.array(tempVal[1:2]) 
        lidar_left = np.array(tempVal[2:3])  
        if(lidar_right.min() < 1):
          if self.c % 5 == 0:    
            self.yaw= self.yaw - 0.1
        if(lidar_left.min() < 1):
          if self.c % 5 == 0:    
            self.yaw= self.yaw + 0.1

    def odometry(self,msg):              
        self.received_x = msg.x
        self.received_y = msg.y
        self.received_z = msg.z        
        if(abs(msg.z)>.75* abs(self.hoverHeight)):                  
            self.reached_Altitude=True
        else:            
            self.reached_Altitude=False

    def vehicleStatus(self,msg):        
        if msg.takeoff_time>0 and self.reached_Altitude== True :
            self.takeoff_success = True
            self.loop_through_gate()             
        else:
            self.takeoff_success = False

    def sub_callback(self, msg):
      self.timestamp = msg.timestamp

    def run(self):      
      if(self.notFound):                
            self.yaw = self.yaw + 0.03
      if(self.yaw >3.14):
                self.yaw = -3.14
      elif(self.yaw <-3.14):
                self.yaw = 3.14
           
      if(self.takeoff_success == False or self.land_at_point == True):        
        self.trajectorysetpointHome()         
        if(self.land_at_point == True and abs(self.received_x) <0.5 and abs(self.received_y) < 0.5):
            self.land()
      else:
          if(self.disableHover == False):
            self.trajectorysetpoint_Hover()          

      if self.c == 20:            
            self.vehicle_command(VehicleCommand().VEHICLE_CMD_DO_SET_MODE,1.0,6.0)
            self.arm()          
      self.c += 1

    def arm(self):
      self.vehicle_command(VehicleCommand().VEHICLE_CMD_COMPONENT_ARM_DISARM,1.0,0.0)

    def disarm(self):
      self.vehicle_command(VehicleCommand().VEHICLE_CMD_COMPONENT_ARM_DISARM,0.0,0.0)

    def land(self):
        print('landing')        
        self.vehicle_command(VehicleCommand().VEHICLE_CMD_NAV_LAND,NaN,NaN)        
        rclpy.shutdown()

    def trajectorysetpointHome(self):
    #   print('self.trajectorysetpoint_Home()')
      msg = TrajectorySetpoint()
      msg.timestamp = self.timestamp
      msg.x = 0.0
      msg.y = 0.0
      msg.z = self.hoverHeight
      msg.vx = 0.1
      msg.vy = 0.1
      self.trajectory_setpoint_publisher.publish(msg)       
      self.offboard_control_mode(True,False)       
    
    def trajectorysetpoint_fwd(self):
      print('self.trajectorysetpoint_Gates_fwd()')
      msg = TrajectorySetpoint()
      msg.timestamp = self.timestamp
      msg.x = NaN
      msg.y = NaN
      msg.z = self.hoverHeight
      msg.vx = self.movementSpeedFactor*math.cos(self.yaw)
      msg.vy = self.movementSpeedFactor*math.sin(self.yaw)
      msg.yaw = self.yaw      
      self.trajectory_setpoint_publisher.publish(msg)       
      self.offboard_control_mode(False,True)       


    def trajectorysetpoint_Hover(self):
      # print('self.trajectorysetpoint_Gates_hover()')
      msg = TrajectorySetpoint()
      msg.timestamp = self.timestamp
      msg.x = NaN
      msg.y = NaN
      msg.z = self.hoverHeight
      msg.vx = 0.01*np.sin(self.yaw)
      msg.vy = 0.01*np.cos(self.yaw)
      msg.vz = 0.0
      msg.yaw = self.yaw      
      self.trajectory_setpoint_publisher.publish(msg)       
      self.offboard_control_mode(False,True)   

    def trajectorysetpoint_Gates_up(self):
      print('self.trajectorysetpoint_Gates_up()')
      msg = TrajectorySetpoint()
      msg.timestamp = self.timestamp
      msg.x = NaN
      msg.y = NaN
      msg.z = self.upHeight
      msg.vx = 0.0
      msg.vy = 0.0
      msg.vz = 1.0
      msg.yaw = self.yaw      
      self.trajectory_setpoint_publisher.publish(msg)       
      self.offboard_control_mode(False,True)       
    
    def trajectorysetpoint_Gates_down(self):
      print('self.trajectorysetpoint_Gates_down()')
      msg = TrajectorySetpoint()
      msg.timestamp = self.timestamp
      msg.x = NaN
      msg.y = NaN
      msg.z = self.hoverHeight
      msg.vx = 0.0
      msg.vy = 0.0
      msg.vz = 1.0
      msg.yaw = self.yaw      
      self.trajectory_setpoint_publisher.publish(msg)       
      self.offboard_control_mode(False,True)       

    def trajectorysetpoint_Gates_reverse(self):
      print('self.trajectorysetpoint_Gates_rev()')
      msg = TrajectorySetpoint()
      msg.timestamp = self.timestamp
      msg.x = NaN
      msg.y = NaN
      msg.z = NaN
      msg.vx = -self.movementSpeedFactor*math.cos(self.yaw)
      msg.vy = -self.movementSpeedFactor*math.sin(self.yaw)
      msg.vz = NaN
      msg.yaw = self.yaw      
      self.trajectory_setpoint_publisher.publish(msg)       
      self.offboard_control_mode(False,True)       
 

    def offboard_control_mode(self,pos,vel):
      msg = OffboardControlMode()    
      msg.timestamp = self.timestamp
      msg.position = pos
      msg.velocity = vel
      msg.acceleration = False
      msg.attitude = False
      msg.body_rate = False      
      self.control_mode_publisher.publish(msg)
      	

    def vehicle_command(self, command, param1, param2):
      msg = VehicleCommand()
      msg.timestamp = self.timestamp
      msg.param1 = param1
      msg.param2 = param2
      msg.command = command
      msg.target_system = 1
      msg.target_component = 1
      msg.source_system = 1
      msg.source_component = 1
      msg.from_external = True
      self.vehicle_command_publisher.publish(msg)   

    def loop_through_gate(self):        
        print('distance moved from',self.received_x_last,' , ',self.received_y_last,' ',math.hypot( self.received_x-self.received_x_last,self.received_y-self.received_y_last),'at yaw ',self.yaw)
        if(self.takeoff_success ==True and self.step == 0):
            self.disableHover = True        
            self.trajectorysetpoint_fwd()        
            if(abs(math.hypot( self.received_x-self.received_x_last,self.received_y-self.received_y_last))>10 ):                
                self.trajectorysetpoint_Gates_up()                                       
                if(abs(self.received_z)>.7* abs(self.upHeight) ):
                    self.step = 2

        if(self.step == 2):            
            self.trajectorysetpoint_Gates_reverse()                       
            if(abs(math.hypot( self.received_x-self.received_x_last,self.received_y-self.received_y_last)) < 5 ):                       
                self.trajectorysetpoint_Gates_down()       
                if(abs(self.received_z)<1.25* abs(self.hoverHeight) ):                    
                    if(abs(math.hypot( self.received_x-self.received_x_last,self.received_y-self.received_y_last))<5 ):
                        self.step = 3

        if(self.step == 3):            
            self.trajectorysetpoint_fwd()        
            if(abs(math.hypot( self.received_x-self.received_x_last,self.received_y-self.received_y_last))>10 ):
                self.getLastPos = True                
                self.yaw = self.yaw - 1.57                              
                self.step = 4
        
        if(self.step == 4):           
            if self.getLastPos == True :
                self.received_x_last = self.received_x        
                self.received_y_last = self.received_y 
            self.getLastPos = False 
            self.trajectorysetpoint_fwd()    
            if(abs(math.hypot( self.received_x-self.received_x_last,self.received_y-self.received_y_last))>2 ):
                self.notFound = True
                self.disableHover = False
                self.color_number = self.color_number + 1
                if(self.color_number >= self.no_color_list):
                  self.land_at_point = True
                  self.color_number = 0
                self.step = 10                
            print('passed through gate,Going fo next')                    
                                    

    def img_cb(self, msg):
        cv_bridge = CvBridge()
        cv_frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")                
        if(self.land_at_point == True):
          out_msg = cv_bridge.cv2_to_imgmsg(cv_frame)      
        else:
          out_msg = cv_bridge.cv2_to_imgmsg(self.find_bricks(cv_frame))      
        out_msg.header.frame_id = msg.header.frame_id        
        self.pub.publish(out_msg)            

    def find_bricks(self, msg):        
        
        lower_h = 0
        upper_h = 180
        lower_s = 48
        upper_s = 255
        lower_v = 0
        upper_v = 255        

        # blur the image with a 3x3 kernel to remove noise
        frame_blur = cv2.blur(msg, (3, 3))        

        # convert to HSV and apply HSV threshold to image
        frame_hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)
        frame_thr = cv2.inRange(frame_hsv, (lower_h, lower_s, lower_v), (upper_h, upper_s, upper_v))

        lower_blue = np.array([110,100,100])
        upper_blue = np.array([130,255,255])

        lower_green = np.array([31,100,50])
        upper_green = np.array([70,255,255])

        lower_Magenta = np.array([140,100,100])
        upper_Magenta = np.array([160,255,255])

        lower_red = np.array([0,50,50])
        upper_red = np.array([20,255,255])

        lower_yellow = np.array([10,100,0])
        upper_yellow = np.array([30,255,255])

        mask_blue = cv2.inRange(frame_hsv, lower_blue, upper_blue)
        mask_Magenta = cv2.inRange(frame_hsv, lower_Magenta, upper_Magenta)
        mask_red = cv2.inRange(frame_hsv, lower_red, upper_red)
        mask_yellow = cv2.inRange(frame_hsv, lower_yellow, upper_yellow)
        mask_green = cv2.inRange(frame_hsv, lower_green, upper_green)        

        colors_mask_list = [mask_blue,mask_green,mask_Magenta,mask_red,mask_yellow]        

        self.no_color_list =len(colors_mask_list) 

        # res_Magenta = cv2.bitwise_and(msg,msg, mask= mask_Magenta)
        # res_red = cv2.bitwise_and(msg,msg, mask= mask_red)
        # res_yellow = cv2.bitwise_and(msg,msg, mask= mask_yellow)
        # res_blue = cv2.bitwise_and(msg,msg, mask= mask_blue)
        # res_green = cv2.bitwise_and(msg,msg, mask= mask_green)

        if self.land_at_point == False:
          mask_color = colors_mask_list[self.color_number]          
          cv2.circle(msg, (160, 120), int(7), (0, 0, 255), -1)
          contours, _ = cv2.findContours(mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                  
          for contour in contours:
            (x,y),radius = cv2.minEnclosingCircle(contour)
            a,b,w,h = cv2.boundingRect(contour)                
            width = w
            height = h            
            if width > 10.0 and height > 10.0 and self.takeoff_success== True and self.step == 10 :                
                if 1/3 <= width/height <= 1 or 1/3 <= height/width <= 1 :
                    cv2.circle(msg, (int(x), int(y)), int(7), (255, 255, 255), -1)
                    cv2.circle(msg, (int(a), int(b+h)), int(5), (255, 0, 255), -1)
                    cv2.rectangle(msg,(a,b),(a+w,b+h),(0,255,0),2)
                    imageWidth = msg.shape[1]
                    imageheight = msg.shape[0]                                    
                    if ((a) <=  imageWidth*3/5 and (a) >=  imageWidth*2/5 ):                    
                        self.getLastPos = True
                        if self.getLastPos == True :
                            self.received_x_last = self.received_x        
                            self.received_y_last = self.received_y                         
                        if (width > 100 and width < 160 and height >150 and height < 230) :
                            self.notFound = False
                            self.getLastPos = False
                            self.step = 0
                            self.yaw = self.yaw +0.5                            
                            self.loop_through_gate()                            
                        elif width > 160 and height > 230:                            
                            self.trajectorysetpoint_Gates_reverse()                            

                        else:                             
                            self.notFound = False                            
                            self.yaw = self.yaw
                            self.getLastPos = False
                            self.trajectorysetpoint_fwd()                            
                           
                    elif ( a>imageWidth*0.5/5 and a< imageWidth*2/5 ):                        
                        if self.c % 5 == 0:    
                            self.yaw = self.yaw  - 0.1
                    else:                                                
                        self.notFound = True                        
                        self.getLastPos = False
          return msg                    
        

def main(args=None):
   rclpy.init(args=args)
   ctrl = OffboardControl()
   try:
        rclpy.spin(ctrl)
   except KeyboardInterrupt:
      pass
   rclpy.shutdown()

if __name__=='__main__':
   main()
