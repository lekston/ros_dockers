#!/usr/bin/env python3

import itertools
import rospy
import numpy as np
from sensor_msgs.msg import Imu


class ImuTimeStatsMonitor:
    def __init__(self):
        rospy.init_node('imu_frame_remapper', anonymous=True)

        # Get parameters with defaults
        self.input_topic = rospy.get_param('~input_topic', '/imu_raw')

        # Setup subscriber and publisher
        self.imu_sub = rospy.Subscriber(self.input_topic, Imu, self.imu_callback, queue_size=10)
        self._buffer_len = 20
        self._msgs = []
        self._start_time = 0

        rospy.loginfo("IMU monitor started:")
        rospy.loginfo("  Input topic: %s", self.input_topic)
        rospy.loginfo("  Size of averaging window: %s", self._buffer_len)

    def timestamp(self, header_time_stamp):
        ts, tn = header_time_stamp.secs, header_time_stamp.nsecs
        return ts + tn*1e-9

    def _pairwise(self, iterable):
        a, b = itertools.tee(iterable)
        next(b, None)
        return zip(a, b)

    def _time_diff(self, m1, m2):
        t1 = self.timestamp(m1.header.stamp)
        t2 = self.timestamp(m2.header.stamp)
        return t2-t1

    def imu_callback(self, msg):
        if self._start_time == 0:
            self._start_time = msg.header.stamp.secs
        msg.header.stamp.secs -= self._start_time
        self._msgs.append(msg)
        if len(self._msgs) > self._buffer_len:
            self._msgs.pop(0)
            # import pdb; pdb.set_trace()
            time_diffs = list(map(lambda p: self._time_diff(p[0], p[1]), self._pairwise(self._msgs)))
            avg = np.average(time_diffs)
            median = np.median(time_diffs)
            std = np.std(time_diffs)
            td_max = max(time_diffs)
            td_min = min(time_diffs)

            rospy.loginfo(
                    f"Time diffs: last {time_diffs[-1]:.6f} [sec]"
                    f"(windowed: min {td_min:.6f}, max {td_max:.6f}, avg {avg:.6f}, median {median:.6f}, std {std:.6f})"
            )


if __name__ == '__main__':
    try:
        remapper = ImuTimeStatsMonitor()
        rospy.loginfo("IMU time stats monitor node is running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
