import os
from definitions import ROOT_DIR


class RawData:
    """Base class to initialize the directories for the raw data

    USAGE:
            RawData('2013-01-10')

    """
    def __init__(self, date):
        if isinstance(date, str):
            self.date = date
        else:
            raise TypeError('"date" must be of type string')

        self.data_dir = ROOT_DIR + '/raw_data/' + str(self.date)

        if os.path.exists(self.data_dir):

            self.ground_truth_dir = self.data_dir + '/ground_truth'
            self.ground_truth_covariance_dir = self.data_dir + '/ground_truth_covariance'
            self.hokuyo_data_dir = self.data_dir + '/hokuyo_data'
            self.sensor_data_dir = self.data_dir + '/sensor_data'
            self.velodyne_data_dir = self.data_dir + '/velodyne_data'
            self.images_dir = self.data_dir + '/images'

        else:
            raise ValueError("raw_data directory not exists")
