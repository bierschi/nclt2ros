import rospy
import struct
import os
import numpy as np
import geometry_msgs.msg
import tf2_msgs.msg
import tf.transformations

from sensor_msgs.msg import PointCloud2, PointField
from nclt2ros.extractor.base_raw_data import BaseRawData
from nclt2ros.converter.base_convert import BaseConvert


class VelodyneSyncData(BaseRawData, BaseConvert):
    """Class to transform the velodyne_sync binary files to ROS PointCloud2 messages

    USAGE:
            VelodyneSyncData('2013-01-10')

    """
    def __init__(self, date):

        # init base classes
        BaseRawData.__init__(self, date=date)
        BaseConvert.__init__(self, date=date)

    def get_velodyne_sync_timestamps_and_files(self):
        """returns the timestamps and binary files in a sorted manner

        :return: timestamps_microsec: List, containing the sorted timestamps
                           bin_files: List, containing the sorted binary files
        """
        if self.velodyne_data_flag:
            files = os.listdir(self.velodyne_sync_data_dir)
        else:
            raise ValueError('velodyne_data not exists')

        timestamps_microsec = sorted([long(os.path.splitext(f)[0]) for f in files if f.endswith('.bin')])
        bin_files = sorted([f for f in files if f.endswith('.bin')])

        return timestamps_microsec, bin_files

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

    def read_next_velodyne_sync_packet(self, file):
        """reads one velodyne sync binary file, equals one revolution of the velodyne sensor

        :param file: one binary file from the velodyne_sync directory

        :return: hits: list, containing the x, y, z, intensity, laser_id of the pointcloud
        """

        try:
            os.chdir(self.velodyne_sync_data_dir)
            f_bin = open(file, "r")

            hits = []
            while True:

                x_str = f_bin.read(2)
                if x_str == '':  # eof
                    break

                x         = struct.unpack('<H', x_str)[0]
                y         = struct.unpack('<H', f_bin.read(2))[0]
                z         = struct.unpack('<H', f_bin.read(2))[0]
                intensity = struct.unpack('B', f_bin.read(1))[0]
                laser_id  = struct.unpack('B', f_bin.read(1))[0]

                x, y, z = self.convert_velodyne(x, y, z)
                y = y * -1
                z = z * -1
                hits += [x, y, z, intensity, laser_id]

            f_bin.close()

            return hits

        except Exception as e:
            print(e)

    def xyzil_array_to_pointcloud2(self, utime, hits):
        """reads the x, y, z, intensity, laser_id list and convert it into a pointcloud2 message

        :param utime: timestamp in microseconds from the binary file
        :param  hits: list, containing the x, y, z intensity, laser_id

        :return: timestamp: ros time object,
                   pc2_msg: pointcloud2 message
        """

        # create a ros timestamp
        timestamp = rospy.Time.from_sec(utime / 1e6)
        points = np.array(hits)

        # get velodyne and base link
        velodyne_link = self.velodyne_frame
        base_link = self.body_frame

        # create a PointCloud2 message
        pc2_msg = PointCloud2()
        pc2_msg.header.stamp = timestamp
        pc2_msg.header.frame_id = velodyne_link

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

        # create base_link velodyne_link static transformer
        vel_static_transform_stamped = geometry_msgs.msg.TransformStamped()
        vel_static_transform_stamped.header.stamp = timestamp
        vel_static_transform_stamped.header.frame_id = base_link
        vel_static_transform_stamped.child_frame_id = velodyne_link

        vel_static_transform_stamped.transform.translation.x = 0.002
        vel_static_transform_stamped.transform.translation.y = 0.004
        vel_static_transform_stamped.transform.translation.z = 0.957

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        vel_static_transform_stamped.transform.rotation.x = quat[0]
        vel_static_transform_stamped.transform.rotation.y = quat[1]
        vel_static_transform_stamped.transform.rotation.z = quat[2]
        vel_static_transform_stamped.transform.rotation.w = quat[3]

        # publish static transform
        tf_static_msg = tf2_msgs.msg.TFMessage([vel_static_transform_stamped])

        return timestamp, pc2_msg, tf_static_msg
