import os
import tarfile
from raw_data import RawData


class ExtractRawData(RawData):
    """Class to extract the raw data

    USAGE:
            ExtractRawData('2013-01-10')

    """

    def __init__(self, date):

        RawData.__init__(self, date=date)
        self.extract_data()

    def extract_data(self):
        """extracts the data from tar.gz format

        """

        # check hokuyo_data
        if os.path.exists(self.hokuyo_data_dir):
            os.chdir(self.hokuyo_data_dir)
            files = os.listdir(self.hokuyo_data_dir)
            for file in files:
                if file.endswith('tar.gz'):
                    print("unpacking {}".format(file))
                    tar = tarfile.open(file, 'r:gz')
                    tar.extractall()
                    tar.close()

        # check sensor_data
        if os.path.exists(self.sensor_data_dir):
            os.chdir(self.sensor_data_dir)
            files = os.listdir(self.sensor_data_dir)
            for file in files:
                if file.endswith('tar.gz'):
                    print("unpacking {}".format(file))
                    tar = tarfile.open(file, 'r:gz')
                    tar.extractall()
                    tar.close()

        # check velodyne data
        if os.path.exists(self.velodyne_data_dir):
            os.chdir(self.velodyne_data_dir)
            files = os.listdir(self.velodyne_data_dir)
            for file in files:
                if file.endswith('tar.gz'):
                    print("unpacking {}".format(file))
                    tar = tarfile.open(file, 'r:gz')
                    tar.extractall()
                    tar.close()

        # check image data
        if os.path.exists(self.images_dir):
            os.chdir(self.images_dir)
            files = os.listdir(self.images_dir)
            for file in files:
                if file.endswith('tar.gz'):
                    print("unpacking {}".format(file))
                    tar = tarfile.open(file, 'r:gz')
                    tar.extractall()
                    tar.close()
