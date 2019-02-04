import os
import json
from definitions import ROOT_DIR


class Data:
    """Base class for data transformation

    USAGE:
            Data('2013-01-10')

    """
    def __init__(self, date):

        if isinstance(date, str):
            self.date = date
        else:
            raise TypeError('"date" must be of type string')

        self.raw_data_dir = ROOT_DIR + '/raw_data/' + str(self.date)

        if os.path.exists(self.raw_data_dir):
            self.ground_truth_dir       = self.raw_data_dir + '/ground_truth'
            self.sensor_data_dir        = self.raw_data_dir + '/sensor_data'
            self.hokuyo_data_dir        = self.raw_data_dir + '/hokuyo_data'
            self.velodyne_data_dir      = self.raw_data_dir + '/velodyne_data'
            self.velodyne_sync_data_dir = self.raw_data_dir + '/velodyne_data/' + '%s' % self.date + '/velodyne_sync/'
            self.images_dir             = self.raw_data_dir + '/images/' + '%s' % self.date + '/lb3/'
        else:
            raise ValueError('path %s does not exist' % self.raw_data_dir)

        with open(ROOT_DIR + '/cfg/configuration.json') as json_configs:
            self.json_configs = json.load(json_configs)

