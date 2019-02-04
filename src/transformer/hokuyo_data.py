import struct
import numpy as np
import rospy
import rosbag
from sensor_msgs.msg import LaserScan
from src.transformer.data import Data


class HokuyoData(Data):
    """Class to transform the hokuyo binary data to ROS LaserScan messages

    USAGE:
            HokuyoData('2013-01-10', write_to_bag=False)

    """
    def __init__(self, date, write_to_bag=False):

        # init base class
        Data.__init__(self, date=date)

        # load binary files
        self.f_bin_hokuyo_4m  = open(self.hokuyo_data_dir + '/%s/hokuyo_4m.bin' % self.date, 'r')
        self.f_bin_hokuyo_30m = open(self.hokuyo_data_dir + '/%s/hokuyo_30m.bin' % self.date, 'r')

        self.write_to_bag = write_to_bag

        if self.write_to_bag is True:
            self.bag = rosbag.Bag('hokuyo.bag', 'w')

    def __del__(self):
        """destructor"""

        self.f_bin_hokuyo_4m.close()
        self.f_bin_hokuyo_30m.close()

    def convert_hokuyo(self, x_s):
        """converts the hokuyo binary data to correct values, check out the paper http://robots.engin.umich.edu/nclt/nclt.pdf

        :param x_s: x value from binary file

        :return: corrected x value
        """

        scaling = 0.005
        offset = -100.0

        x = x_s * scaling + offset

        return x

    def read_next_hokuyo_4m_packet(self):
        """reads the hokuyo 4m packets from binary file

        :return: utime: time in microseconds
                     r: np array containing one observation
        """

        try:
            num_hits = 726

            utime_str = self.f_bin_hokuyo_4m.read(8)
            if utime_str == '':  # EOF reached
                return -1, None

            utime = struct.unpack('<Q', utime_str)[0]

            r = np.zeros(num_hits)
            for i in range(num_hits):
                s = struct.unpack('<H', self.f_bin_hokuyo_4m.read(2))[0]
                r[i] = self.convert_hokuyo(s)

            return utime, r

        except Exception as e:
            print(e)

    def write_hokuyo_4m_to_laserscan(self, utime, data):
        """writes the hokuyo data into a LaserScan message

        :param utime: timestamp in microseconds
        :param  data: list containing the data

        :return: timestamp: ros timestamp
                      scan: LaserScan message
        """

        try:
            # create a ros timestamp
            timestamp = rospy.Time.from_sec(utime / 1e6)

            # create a LaserScan message
            scan = LaserScan()
            scan.header.stamp = timestamp
            scan.header.frame_id = self.json_configs['frame_ids']['hokuyo_urg_lidar']

            scan.angle_min = -np.pi / 2
            scan.angle_max = np.pi / 2
            scan.angle_increment = 0.0061359233
            scan.time_increment = 1.466e-4
            scan.scan_time = 0.1
            scan.range_min = 0
            scan.range_max = 100.0

            scan.ranges = data

            return timestamp, scan

        except Exception as e:
            print(e)

    def read_next_hokuyo_30m_packet(self):
        """reads the hokuyo 30m packets from the binary file

        :return: utime: time in microseconds
                     r: np array containing one observation
        """

        try:
            num_hits = 1081

            utime_str = self.f_bin_hokuyo_30m.read(8)
            if utime_str == '':  # EOF reached
                return -1, None

            utime = struct.unpack('<Q', utime_str)[0]

            r = np.zeros(num_hits)
            for i in range(num_hits):
                s = struct.unpack('<H', self.f_bin_hokuyo_30m.read(2))[0]
                r[i] = self.convert_hokuyo(s)

            return utime, r

        except Exception as e:
            print(e)

    def write_hokuyo_30m_to_laserscan(self, utime, data):
        """writes the data into a LaserScan message

        :param utime: timestamp in microseconds
        :param  data: list containing the data

        :return: timestamp: ros timestamp
                      scan: LaserScan message
        """
        try:
            # create ros timestamp
            timestamp = rospy.Time.from_sec(utime / 1e6)

            # create LaserScan message
            scan = LaserScan()
            scan.header.stamp = timestamp
            scan.header.frame_id = self.json_configs['frame_ids']['hokuyo_utm_lidar']

            scan.angle_min = -np.pi / 2
            scan.angle_max = np.pi / 2
            scan.angle_increment = 0.004363
            scan.time_increment = 9.250e-5
            scan.scan_time = 0.1
            scan.range_min = 0
            scan.range_max = 100.0

            scan.ranges = data

            return timestamp, scan

        except Exception as e:
            print(e)
