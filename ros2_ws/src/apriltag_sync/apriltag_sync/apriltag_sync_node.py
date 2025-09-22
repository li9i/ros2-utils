#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, CameraInfo

class AprilSensorSynchronizer(Node):
    def __init__(self):
        super().__init__('april_sensor_sync')

        # Declare and get parameters
        self.declare_parameter('image_topic_in', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic_in', '/camera/color/camera_info')
        self.declare_parameter('image_topic_out', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic_out', '/camera/color/camera_info')

        self.rgb_topic_in = self.get_parameter('image_topic_in').get_parameter_value().string_value
        self.camera_info_topic_in = self.get_parameter('camera_info_topic_in').get_parameter_value().string_value
        self.rgb_topic_out = self.get_parameter('image_topic_out').get_parameter_value().string_value
        self.camera_info_topic_out = self.get_parameter('camera_info_topic_out').get_parameter_value().string_value

        # Publishers
        self.synced_rgb_pub = self.create_publisher(Image, self.rgb_topic_out, 10)
        self.synced_camera_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic_out, 10)

        # Subscribers (via message_filters)
        self.rgb_image_sub = Subscriber(self, Image, self.rgb_topic_in)
        self.camera_info_sub = Subscriber(self, CameraInfo, self.camera_info_topic_in)

        # Synchronizer
        ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.camera_info_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.callback)

    def callback(self, rgb_msg, camera_info_msg):
        self.get_logger().info("april sync-ing")

        synced_camera_info_msg = camera_info_msg
        synced_camera_info_msg.header = rgb_msg.header

        self.synced_rgb_pub.publish(rgb_msg)
        self.synced_camera_info_pub.publish(synced_camera_info_msg)


def main(args=None):
    rclpy.init(args=args)
    synchronizer = AprilSensorSynchronizer()
    rclpy.spin(synchronizer)
    synchronizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
