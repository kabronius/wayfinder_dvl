#TODO: Add comments and license
import time
import math
import datetime as dt
from typing import List
import numpy as np
import rclpy
import signal
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from dvl.dvl import Dvl
from dvl.system import OutputData
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Range
from wayfinder_dvl_msgs.msg import WayfinderDVL

from rclpy.parameter import Parameter

class WayfinderDVLNode(Node):
    def __init__(self) -> None:
        super().__init__('wayfinder_dvl')

        self.declare_parameter("device_port","/dev/ttyUSB0")
        self.declare_parameter("sensor_frame_id","dvl")

        port = self.get_parameter("device_port").get_parameter_value().string_value
        self.sensor_frame_id = self.get_parameter("sensor_frame_id").get_parameter_value().string_value
        
        self.sensor_setup(port)

        self.twist_data_pub = self.create_publisher(TwistWithCovarianceStamped,
                                               "/wayfinder/twist", qos_profile_system_default)

        self.range1_data_pub = self.create_publisher(Range,"/wayfinder/beam_1/range",
                                                     qos_profile_system_default)
        self.range2_data_pub = self.create_publisher(Range,"/wayfinder/beam_2/range",
                                                     qos_profile_system_default)
        self.range3_data_pub = self.create_publisher(Range,"/wayfinder/beam_3/range",
                                                     qos_profile_system_default)
        self.range4_data_pub = self.create_publisher(Range,"/wayfinder/beam_4/range",
                                                     qos_profile_system_default)
        
        self.dvl_data_pub = self.create_publisher(WayfinderDVL,"wayfinder/dvl_data",
                                                  qos_profile_system_default)
        
        self.twist_msg = TwistWithCovarianceStamped()
        self.twist_msg.header.frame_id = self.sensor_frame_id

        self.range1_msg = Range()
        self.range1_msg.header.frame_id = self.sensor_frame_id
        self.range1_msg.radiation_type = Range.ULTRASOUND
        self.range1_msg.field_of_view = 0.5236 #30°
        self.range1_msg.max_range = 60.0 #m
        self.range1_msg.max_range = 0.5 #m

        self.range2_msg = Range()
        self.range2_msg.header.frame_id = self.sensor_frame_id
        self.range2_msg.radiation_type = Range.ULTRASOUND
        self.range2_msg.field_of_view = 0.5236 #30°
        self.range2_msg.max_range = 60.0 #m
        self.range2_msg.max_range = 0.5 #m

        self.range3_msg = Range()
        self.range3_msg.header.frame_id = self.sensor_frame_id
        self.range3_msg.radiation_type = Range.ULTRASOUND
        self.range3_msg.field_of_view = 0.5236 #30°
        self.range3_msg.max_range = 60.0 #m
        self.range3_msg.max_range = 0.5 #m
        
        self.range4_msg = Range()
        self.range4_msg.header.frame_id = self.sensor_frame_id
        self.range4_msg.radiation_type = Range.ULTRASOUND
        self.range4_msg.field_of_view = 0.5236 #30°
        self.range4_msg.max_range = 60.0 #m
        self.range4_msg.max_range = 0.5 #m

        self.dvl_msg = WayfinderDVL()
        
    def sensor_setup(self, device_port: str) -> None:

        self.sensor = Dvl()

        while not self.sensor.connect(device_port, 115200):
            pass

        self.sensor.reset_to_defaults()
        self.sensor.enter_command_mode()
        twosecs = dt.datetime.now() + dt.timedelta(seconds=2)
        timetgt = dt.datetime(twosecs.year, twosecs.month, twosecs.day,
                              twosecs.hour, twosecs.minute, twosecs.second)
        now = dt.datetime.now()
        time.sleep((timetgt - now).total_seconds())

        self.sensor.set_time(dt.datetime.now())

        self.sensor.exit_command_mode()
        self.sensor.register_ondata_callback(self.wayfinderDataCallback, None)
    
    def wayfinderDataCallback(self,dataObj: OutputData, *args) -> None:

        self.twist_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.range1_msg.header.stamp = self.get_clock().now().to_msg()
        self.range2_msg.header.stamp = self.get_clock().now().to_msg()
        self.range3_msg.header.stamp = self.get_clock().now().to_msg()
        self.range4_msg.header.stamp = self.get_clock().now().to_msg()

        self.dvl_msg.header.stamp = self.get_clock().now().to_msg()
        self.dvl_msg.coordinate_system = dataObj.coordinate_system
        self.dvl_msg.speed_of_sound = dataObj.speed_of_sound

        if math.isnan(dataObj.vel_x) or math.isnan(dataObj.vel_y) or \
                math.isnan(dataObj.vel_z) or math.isnan(dataObj.vel_err):
            self.get_logger().error("NaN velocities\n")
        
        if dataObj.is_velocity_valid():
            self.twist_msg.twist.twist.linear.x = dataObj.vel_x
            self.twist_msg.twist.twist.linear.y = dataObj.vel_y
            self.twist_msg.twist.twist.linear.z = dataObj.vel_z
            
            self.twist_data_pub.publish(self.twist_msg)

            self.dvl_msg.velocity.x = dataObj.vel_x
            self.dvl_msg.velocity.y = dataObj.vel_y
            self.dvl_msg.velocity.z = dataObj.vel_z
        else:
            self.get_logger().error("Invalid velocities\n")

        if math.isnan(dataObj.range_beam1) or math.isnan(dataObj.range_beam2) or \
                math.isnan(dataObj.range_beam3) or math.isnan(dataObj.range_beam4):
            self.get_logger().error("NaN beams ranges\n")
            
        if dataObj.is_range_valid():
            self.range1_msg.range = dataObj.range_beam1
            self.range2_msg.range = dataObj.range_beam2
            self.range3_msg.range = dataObj.range_beam3                
            self.range4_msg.range = dataObj.range_beam4
            
            self.range1_data_pub.publish(self.range1_msg)
            self.range2_data_pub.publish(self.range2_msg)
            self.range3_data_pub.publish(self.range3_msg)
            self.range4_data_pub.publish(self.range4_msg)

            self.dvl_msg.range_beam1 = dataObj.range_beam1
            self.dvl_msg.range_beam2 = dataObj.range_beam2
            self.dvl_msg.range_beam3 = dataObj.range_beam3
            self.dvl_msg.range_beam4 = dataObj.range_beam4

            self.dvl_msg.mean_range = dataObj.mean_range
        else:
            self.get_logger().error("Invalid ranges\n")

def main(args = None):

    rclpy.init(args=args)

    node = WayfinderDVLNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
