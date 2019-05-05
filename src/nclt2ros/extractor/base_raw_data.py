import os
import rospy
from nclt2ros.definitions import ROOT_DIR


class BaseRawData:
    """Base class to initialize the directories for the raw data

    USAGE:
            BaseRawData('2013-01-10')

    """
    def __init__(self, date):

        self.date = date

        RAW_DATA_PATH_DFLT = ROOT_DIR + '/raw_data/'
        #RAW_DATA_PATH_DFLT = '/home/christian/projects/nclt2rosbag/nclt2rosbag/raw_data'
        raw_data_path = rospy.get_param('~raw_data_path', RAW_DATA_PATH_DFLT)

        self.raw_data_path = raw_data_path
        # init raw directory
        if self.raw_data_path.endswith('/'):
            self.raw_data_dir = self.raw_data_path + str(self.date)
        else:
            self.raw_data_dir = self.raw_data_path + '/' + str(self.date)

        # check if data exists
        if os.path.exists(self.raw_data_dir):

            self.ground_truth_dir = self.raw_data_dir + '/ground_truth'
            if os.path.exists(self.ground_truth_dir):
                self.ground_truth_flag = True
            else:
                self.ground_truth_flag = False

            self.ground_truth_covariance_dir = self.raw_data_dir + '/ground_truth_covariance'
            if os.path.exists(self.ground_truth_covariance_dir):
                self.ground_truth_covariance_flag = True
            else:
                self.ground_truth_covariance_flag = False

            self.hokuyo_data_dir = self.raw_data_dir + '/hokuyo_data'
            if os.path.exists(self.hokuyo_data_dir):
                self.hokuyo_data_flag = True
            else:
                self.hokuyo_data_flag = False

            self.sensor_data_dir = self.raw_data_dir + '/sensor_data'
            if os.path.exists(self.sensor_data_dir):
                self.sensor_data_flag = True
            else:
                self.sensor_data_flag = False

            self.velodyne_data_dir = self.raw_data_dir + '/velodyne_data'
            if os.path.exists(self.velodyne_data_dir):
                self.velodyne_data_flag = True
                self.velodyne_sync_data_dir = self.raw_data_dir + '/velodyne_data/' + '%s' % self.date + '/velodyne_sync/'
            else:
                self.velodyne_data_flag = False

            self.images_dir = self.raw_data_dir + '/images'
            if os.path.exists(self.images_dir):
                self.images_flag = True
                self.images_lb3_dir = self.raw_data_dir + '/images/' + '%s' % self.date + '/lb3/'
            else:
                self.images_flag = False

        else:
            raise ValueError("raw_data directory not exists")

