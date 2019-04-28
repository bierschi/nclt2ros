import numpy as np
from nclt2ros.extractor.base_raw_data import BaseRawData


class ReadGroundTruth(BaseRawData):
    """Class to read the ground_truth directory

    USAGE:
            ReadGroundTruth('2013-01-10')

    """
    def __init__(self, date):
        BaseRawData.__init__(self, date=date)

    def read_gt_csv(self, all_in_one=None):
        """reads the data in the ground_truth csv file

        :param   all_in_one: Boolean
        :return: ndarray: containing all data, if all_in_one is True
                 else
                 each column seperated as utimes, x, y, z, roll_rad, pitch_rad, yaw_rad
        """

        if self.ground_truth_flag:

            gt = np.loadtxt(self.ground_truth_dir + '/groundtruth_%s.csv' % self.date, delimiter=",")

            if all_in_one is True:

                return gt

            else:
                utimes    = gt[:, 0]
                x         = gt[:, 1]
                y         = gt[:, 2]
                z         = gt[:, 3]
                roll_rad  = gt[:, 4]
                pitch_rad = gt[:, 5]
                yaw_rad   = gt[:, 6]

                return utimes, x, y, z, roll_rad, pitch_rad, yaw_rad
        else:
            raise ValueError('no ground_truth directory available')
