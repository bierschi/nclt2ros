import numpy as np
from nclt2rosbag.extractor.base_raw_data import BaseRawData


class ReadGroundTruthCovariance(BaseRawData):
    """Class to read the ground_truth_covariance directory

    USAGE:
            ReadGroundTruthCovariance('2013-01-10')

    """
    def __init__(self, date):
        BaseRawData.__init__(self, date=date)

    def read_gt_cov_csv(self, all_in_one=None):
        """reads the data in the ground_truth_covariance csv file

        :param   all_in_one: Boolean
        :return: ndarray: containing all data, if all_in_one is True
                 else
                 each column seperated as utimes and covariance matrice ...
        """

        if self.ground_truth_covariance_flag:

            gt_cov = np.loadtxt(self.ground_truth_covariance_dir + '/cov_%s.csv' % self.date, delimiter=",")

            if all_in_one is True:

                return gt_cov

            else:
                utimes = gt_cov[:, 0]
                xx = gt_cov[:, 1];  xy = gt_cov[:, 2];  xz = gt_cov[:, 3];  xr = gt_cov[:, 4];  xp = gt_cov[:, 5]; xh = gt_cov[:, 6]
                yy = gt_cov[:, 7];  yz = gt_cov[:, 8];  yr = gt_cov[:, 9];  yp = gt_cov[:, 10]; yh = gt_cov[:, 11]
                zz = gt_cov[:, 12]; zr = gt_cov[:, 13]; zp = gt_cov[:, 14]; zh = gt_cov[:, 15]
                rr = gt_cov[:, 16]; rp = gt_cov[:, 17]; rh = gt_cov[:, 18]
                pp = gt_cov[:, 19]; ph = gt_cov[:, 20]
                hh = gt_cov[:, 21]

                return utimes, xx, xy, xz, xr, xp, xh, yy, yz, yr, yp, yh, zz, zr, zp, zh, rr, rp, rh, pp, ph, hh

        else:
            raise ValueError('no ground_truth_covariance directory available')
