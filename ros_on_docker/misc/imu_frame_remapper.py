#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from typing import List

# NOTE: NTU viral data /imu/imu to LIO-SAM /imu_raw conversion
# INFO this is just NED-to-ENU rotation, however LIO-SAM does not use TF so it has to be applied manualy
# Rx_180


def quat_mult(q1: Quaternion, q2: Quaternion) -> List[float]:
    # s - scalar part
    # v - vector part
    # q_{1}q_{2}=\left(s1+{\vec {v1}}\right)\left(s2+{\vec {v2}}\right)
    # q_{1}q_{2}=\left(s1s2-{\vec {v1}}\cdot {\vec {v2}}\right)
    #           +\left(s1{\vec {v2}}+s2{\vec {v1}}+{\vec {v1}}\times {\vec {v2}}\right)
    s1 = q1.w
    v1 = [q1.x, q1.y, q1.z]
    s2 = q2.w
    v2 = [q2.x, q2.y, q2.z]

    q_out_s = s1 * s2 - np.dot(v1, v2)
    q_out_v = np.dot(s1, v2) + np.dot(s2, v1) + np.cross(v1, v2)
    return [q_out_v[0], q_out_v[1], q_out_v[2], q_out_s]


rot_enu_from_ned = np.ndarray(
    shape=(3,3), buffer=np.array([
        0.0, 1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, -1.0]
    )
)


quat_enu_from_ned = quaternion_from_euler(np.pi, 0, 0)


def transform_quat_enu_from_ned(quat_orig: Quaternion) -> Quaternion:
    """
    Args:
        quat_orig: rotation from body coord frame to ground (NED)
    Output:
        quat_out: rotation from body coord frame to ground (ENU)
    """
    quat_ned_orig = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
    rpy = euler_from_quaternion(quat_ned_orig)
    print(f"Q NED: {quat_ned_orig} // RPY: {rpy}")
    quat_out = quaternion_multiply(quat_enu_from_ned, quat_ned_orig)
    # print(f"Q ENU (build-in): {quat_out}")
    # q_out_raw = quat_mult(Quaternion(*list(quat_enu_from_ned)), quat_orig)
    # quat_out = q_out_raw
    # print(f"Q ENU (own): {q_out_raw}")
    return Quaternion(*list(quat_out))


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
        quat_enu = transform_quat_enu_from_ned(msg.orientation)
        output_msg.orientation = quat_enu
        output_msg.orientation_covariance = msg.orientation_covariance
        output_msg.angular_velocity = msg.angular_velocity
        output_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        output_msg.linear_acceleration = msg.linear_acceleration
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