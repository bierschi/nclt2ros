import os
import subprocess

from nclt2rosbag.definitions import ROOT_DIR


class Download:
    """Class to download the NCLT Dataset from http://robots.engin.umich.edu/nclt/
    USAGE:
            Download(args=args)
    """

    def __init__(self, args):

        print("Download NCLT dataset from %s" % args.date)

        self.download_url_dir = 'http://robots.engin.umich.edu/nclt'
        self.dates = ['2012-01-08', '2012-01-15', '2012-01-22', '2012-02-02', '2012-02-04', '2012-02-05', '2012-02-12',
                      '2012-02-18', '2012-02-19', '2012-03-17', '2012-03-25', '2012-03-31', '2012-04-29', '2012-05-11',
                      '2012-05-26', '2012-06-15', '2012-08-04', '2012-08-20', '2012-09-28', '2012-10-28', '2012-11-04',
                      '2012-11-16', '2012-11-17', '2012-12-01', '2013-01-10', '2013-02-23', '2013-04-05']

        self.args = args
        self.date = args.date

        self.raw_data_dir = ROOT_DIR + '/raw_data/' + str(self.date)
        if not os.path.exists(self.raw_data_dir):
            os.makedirs(self.raw_data_dir)

        self.saved_path = None

        if self.check_date(self.date):
            self.load_dataset()
        else:
            raise ValueError("Given 'Date' is not in dataset")

    def load_dataset(self):
        """
        load data depending on the arguments specified in the argument parser
        """

        self.saved_path = os.getcwd()
        os.chdir(self.raw_data_dir)

        if (not self.args.lb3) and (not self.args.sen) and (not self.args.vel) and (not self.args.hokuyo) and \
           (not self.args.gt) and (not self.args.gt_cov):
            print("Download all data")
            self.get_images()
            self.get_sensors()
            self.get_velodyne()
            self.get_hokuyo()
            self.get_ground_truth_pose()
            self.get_ground_truth_cov()

        if self.args.lb3:
            print("Download Images")
            self.get_images()

        if self.args.sen:
            print("Download Sensors")
            self.get_sensors()

        if self.args.vel:
            print("Download Velodyne")
            self.get_velodyne()

        if self.args.hokuyo:
            print("Download Hokuyo")
            self.get_hokuyo()

        if self.args.gt:
            print("Download Ground Truth")
            self.get_ground_truth_pose()

        if self.args.gt_cov:
            print("Download Ground Truth Covariance")
            self.get_ground_truth_cov()

        os.chdir(self.saved_path)

    def check_date(self, date):
        """
        checks if date is in self.dates
        :param date: String
        :return: Bool, True if date is in self.dates
                       else False
        """

        if date in self.dates:
            return True
        else:
            return False

    def get_images(self):
        """
        fetching the raw image data with wget and the continue flag for later continuation
        """

        cmd = ['wget', '--continue', '%s/images/%s_lb3.tar.gz' % (self.download_url_dir, self.date), '-P', 'images']
        print "Calling: ", ' '.join(cmd)
        subprocess.call(cmd)

    def get_sensors(self):
        """
        fetching the raw sensors data with wget and the continue flag for later continuation
        """

        cmd = ['wget', '--continue', '%s/sensor_data/%s_sen.tar.gz' % (self.download_url_dir, self.date), '-P', 'sensor_data']
        print "Calling: ", ' '.join(cmd)
        subprocess.call(cmd)

    def get_velodyne(self):
        """
        fetching the raw velodyne data with wget and the continue flag for later continuation
        """

        cmd = ['wget', '--continue', '%s/velodyne_data/%s_vel.tar.gz' % (self.download_url_dir, self.date), '-P', 'velodyne_data']
        print "Calling: ", ' '.join(cmd)
        subprocess.call(cmd)

    def get_hokuyo(self):
        """
        fetching the raw hokuyo data with wget and the continue flag for later continuation
        """

        cmd = ['wget', '--continue', '%s/hokuyo_data/%s_hokuyo.tar.gz' % (self.download_url_dir, self.date), '-P', 'hokuyo_data']
        print "Calling: ", ' '.join(cmd)
        subprocess.call(cmd)

    def get_ground_truth_pose(self):
        """
        fetching the raw ground truth pose data with wget and the continue flag for later continuation
        """

        cmd = ['wget', '--continue', '%s/ground_truth/groundtruth_%s.csv' % (self.download_url_dir, self.date), '-P', 'ground_truth']
        print "Calling: ", ' '.join(cmd)
        subprocess.call(cmd)

    def get_ground_truth_cov(self):
        """
        fetching the raw ground truth covariance data with wget and the continue flag for later continuation
        """

        cmd = ['wget', '--continue', '%s/covariance/cov_%s.csv' % (self.download_url_dir, self.date), '-P', 'ground_truth_covariance']
        print "Calling: ", ' '.join(cmd)
        subprocess.call(cmd)