import struct
import numpy as np
import rospy

from sensor_msgs.msg import PointCloud2, PointField
from nclt2ros.extractor.base_raw_data import BaseRawData
from nclt2ros.converter.base_convert import BaseConvert


class VelodyneData(BaseRawData, BaseConvert):
    """Class to convert the velodyne binary file to ROS PointCloud2 messages

    USAGE:
            VelodyneData('2013-01-10')

    """
    def __init__(self, date):

        # init base class
        BaseRawData.__init__(self, date=date)
        BaseConvert.__init__(self, date=date)

        # load velodyne_binary file
        if self.velodyne_data_flag:
            self.f_bin_velodyne   = open(self.velodyne_data_dir + '/%s/velodyne_hits.bin' % self.date, 'r')
        else:
            raise ValueError('velodyne_data directory not exists')

    def verify_magic(self, s):
        """verifies the binary data

        :param s:
        :return: True, if data is correct
                 False
        """

        magic = 44444
        m = struct.unpack('<HHHH', s)

        return (len(m) >= 3) and (m[0] == magic) and (m[1] == magic) and (m[2] == magic) and (m[3] == magic)

    def convert_velodyne(self, x_s, y_s, z_s):
        """converts the velodyne binary data to corrected values, check out the paper http://robots.engin.umich.edu/nclt/nclt.pdf

        :param x_s: x value from binary file
        :param y_s: y value from binary file
        :param z_s: z value from binary file

        :return: converted x, y, z values
        """
        scaling = 0.005
        offset = -100.0

        x = x_s * scaling + offset
        y = y_s * scaling + offset
        z = z_s * scaling + offset

        return x, y, z

    def read_next_velodyne_packet(self):
        """reads the velodyne binary file

        :return: utime:
                  data:
              num_hits:
        """
        try:

            magic = self.f_bin_velodyne.read(8)
            if magic == '':  # EOF reached
                return -1, None

            if not self.verify_magic(magic):
                print "Could not verify magic"
                return -1, None

            num_hits = struct.unpack('<I', self.f_bin_velodyne.read(4))[0]
            utime = struct.unpack('<Q', self.f_bin_velodyne.read(8))[0]

            self.f_bin_velodyne.read(4)  # padding

            data = []
            for i in range(num_hits):
                x         = struct.unpack('<H', self.f_bin_velodyne.read(2))[0]
                y         = struct.unpack('<H', self.f_bin_velodyne.read(2))[0]
                z         = struct.unpack('<H', self.f_bin_velodyne.read(2))[0]
                intensity = struct.unpack('B',  self.f_bin_velodyne.read(1))[0]
                laser_id  = struct.unpack('B',  self.f_bin_velodyne.read(1))[0]

                x, y, z = self.convert_velodyne(x, y, z)
                data += [x, y, z, float(intensity), float(laser_id)]

            return utime, data, num_hits

        except Exception as e:
            print(e)

    def xyzil_array_to_pointcloud2(self, utime, hits):
        """reads the x, y, z, intensity, laser_id list and convert it into a pointcloud2 message

        :param utime: timestamp in microseconds from the binary file
        :param  hits: list, containing the x, y, z intensity, laser_id

        :return: timestamp: ros time object,
                   pc2_msg: pointcloud2 message
        """

        timestamp = rospy.Time.from_sec(utime / 1e6)
        points = np.array(hits)

        pc2_msg = PointCloud2()
        pc2_msg.header.stamp = timestamp
        pc2_msg.header.frame_id = self.velodyne_frame

        num_values = points.shape[0]
        assert(num_values > 0)

        NUM_FIELDS = 5
        assert(np.mod(num_values, NUM_FIELDS) == 0)

        num_points = num_values / NUM_FIELDS

        assert(len(points.shape) == 1)
        pc2_msg.height = 1

        FLOAT_SIZE_BYTES = 4
        pc2_msg.width = num_values * FLOAT_SIZE_BYTES

        pc2_msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('i', 12, PointField.FLOAT32, 1),
            PointField('l', 16, PointField.FLOAT32, 1)
        ]

        pc2_msg.is_bigendian = False
        pc2_msg.point_step = NUM_FIELDS * FLOAT_SIZE_BYTES

        pc2_msg.row_step = pc2_msg.point_step * num_points
        pc2_msg.is_dense = False

        pc2_msg.width = num_points
        pc2_msg.data = np.asarray(points, np.float32).tostring()

        return timestamp, pc2_msg