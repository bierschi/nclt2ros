import os
import rospy
from nclt2ros.definitions import ROOT_DIR


class BaseConvert:
    """Base Class for converting

    USAGE:
            BaseConvert(date='2013-01-10')

    """
    def __init__(self, date):

        self.date = date

        # create rosbag directory
        ROSBAG_PATH = ROOT_DIR + '/rosbags/'
        self.rosbag_path = rospy.get_param('~rosbag_output_path', ROSBAG_PATH)

        if self.rosbag_path.endswith('/'):
            self.rosbag_dir = self.rosbag_path + str(self.date)
        else:
            self.rosbag_dir = self.rosbag_path + '/' + str(self.date)

        if not os.path.exists(self.rosbag_dir):
            os.makedirs(self.rosbag_dir)

        # create camera folder settings
        self.num_cameras = 6

        # init topic names
        self.gps_fix_topic       = rospy.get_param('~gps_fix', '/navsat/fix')
        self.gps_track_topic     = rospy.get_param('~gps_track', '/navsat/track')
        self.gps_speed_topic     = rospy.get_param('~gps_speed', '/navsat/speed')
        self.gps_rtk_fix_topic   = rospy.get_param('~gps_rtk_fix', '/navsat/rtk/fix')
        self.gps_rtk_track_topic = rospy.get_param('~gps_rtk_track', '/navsat/rtk/track')
        self.gps_rtk_speed_topic = rospy.get_param('~gps_rtk_speed', '/navsat/rtk/speed')
        self.imu_data_topic      = rospy.get_param('~ms25_imu_data', '/imu/data')
        self.imu_mag_topic       = rospy.get_param('~ms25_imu_mag', '/imu/mag')
        self.odom_topic          = rospy.get_param('~wheel_odometry_topic', '/odom')
        self.hokuyo_utm_topic    = rospy.get_param('~hokuyo_utm_topic', '/hokuyo_30m')
        self.hokuyo_urg_topic    = rospy.get_param('~hokuyo_urg_topic', '/hokuyo_4m')
        self.velodyne_topic      = rospy.get_param('~velodyne_topic', '/velodyne_points')
        self.ladybug_topic       = rospy.get_param('~ladybug_topic', '/images/raw')
        self.ground_truth_topic  = rospy.get_param('~ground_truth_topic', '/ground_truth')

        # init frame ids
        self.gps_frame = rospy.get_param('~gps_sensor', 'gps_link')
        self.gps_rtk_frame = rospy.get_param('~gps_rtk_sensor', 'gps_rtk_link')
        self.imu_frame = rospy.get_param('~imu_sensor', 'imu_link')
        self.odom_frame = rospy.get_param('~wheel_odometry', 'odom_link')
        self.hok_utm_frame = rospy.get_param('~hokuyo_utm_lidar', 'laser_utm')
        self.hok_urg_frame = rospy.get_param('~hokuyo_urg_lidar', 'laser_urg')
        self.velodyne_frame = rospy.get_param('~velodyne_lidar', 'velodyne')
        self.ladybug_frame = rospy.get_param('~ladybug_sensor', 'camera')
        self.ground_truth_frame = rospy.get_param('~ground_truth', 'ground_truth_link')
        self.body_frame = rospy.get_param('~body', 'base_link')
