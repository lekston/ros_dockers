#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

# NOTE: NTU viral data /imu/imu to LIO-SAM /imu_raw conversion
# INFO this is just NED-to-ENU rotation, however LIO-SAM does not use TF so it has to be applied manualy


class ImuFrameRemapper:
    def __init__(self):
        rospy.init_node('imu_frame_remapper', anonymous=True)

        # Get parameters with defaults
        self.input_topic = rospy.get_param('~input_topic', '/imu/imu')
        self.output_topic = rospy.get_param('~output_topic', '/imu_raw')
        self.new_frame_id = rospy.get_param('~new_frame_id', 'imu_link')

        # Setup subscriber and publisher
        self.imu_sub = rospy.Subscriber(self.input_topic, Imu, self.imu_callback, queue_size=10)
        self.imu_pub = rospy.Publisher(self.output_topic, Imu, queue_size=10)

        rospy.loginfo("IMU Frame Remapper started:")
        rospy.loginfo("  Input topic: %s", self.input_topic)
        rospy.loginfo("  Output topic: %s", self.output_topic)
        rospy.loginfo("  New frame_id: %s", self.new_frame_id)

    def imu_callback(self, msg):
        # Create a copy of the message
        output_msg = Imu()
        output_msg.header = msg.header
        output_msg.orientation = msg.orientation
        output_msg.orientation_covariance = msg.orientation_covariance
        output_msg.angular_velocity = msg.angular_velocity
        output_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        output_msg.linear_acceleration.x = -msg.linear_acceleration.x
        output_msg.linear_acceleration.y = msg.linear_acceleration.y
        output_msg.linear_acceleration.z = -msg.linear_acceleration.z
        output_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        # Update the frame_id
        output_msg.header.frame_id = self.new_frame_id

        # Republish the message
        self.imu_pub.publish(output_msg)

        # Optional: Log the remapping (comment out for performance)
        # rospy.logdebug("Remapped IMU frame_id from '%s' to '%s'",
        #                msg.header.frame_id, self.new_frame_id)


if __name__ == '__main__':
    try:
        remapper = ImuFrameRemapper()
        rospy.loginfo("IMU Frame Remapper node is running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass