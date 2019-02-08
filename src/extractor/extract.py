import os
import tarfile
from raw_data import RawData


class Extract(RawData):
    """Class to extract the raw data

    USAGE:
            ExtractRawData('2013-01-10')

    """

    def __init__(self, date):

        if isinstance(date, str):
            self.date = date
        else:
            raise TypeError("'date' must be type of string")

        RawData.__init__(self, date=date)
        self.extract_data()

    def extract_data(self):
        """extracts the data from tar.gz format

        """

        # check hokuyo_data
        if os.path.exists(self.hokuyo_data_dir):
            os.chdir(self.hokuyo_data_dir)
            files = os.listdir(self.hokuyo_data_dir)
            if not (self.date in files):
                for file in files:
                    if file.endswith('tar.gz'):
                        print("unpacking {}".format(file))
                        tar = tarfile.open(file, 'r:gz')
                        tar.extractall()
                        tar.close()
            else:
                print("hokuyo_data already exist")

        # check sensor_data
        if os.path.exists(self.sensor_data_dir):
            os.chdir(self.sensor_data_dir)
            files = os.listdir(self.sensor_data_dir)
            if not (self.date in files):
                for file in files:
                    if file.endswith('tar.gz'):
                        print("unpacking {}".format(file))
                        tar = tarfile.open(file, 'r:gz')
                        tar.extractall()
                        tar.close()
            else:
                print("sensor_data already exist")

        # check velodyne data
        if os.path.exists(self.velodyne_data_dir):
            os.chdir(self.velodyne_data_dir)
            files = os.listdir(self.velodyne_data_dir)
            if not (self.date in files):
                for file in files:
                    if file.endswith('tar.gz'):
                        print("unpacking {}".format(file))
                        tar = tarfile.open(file, 'r:gz')
                        tar.extractall()
                        tar.close()
            else:
                print("velodyne_data already exist")

        # check image data
        if os.path.exists(self.images_dir):
            os.chdir(self.images_dir)
            files = os.listdir(self.images_dir)
            if not (self.date in files):
                for file in files:
                    if file.endswith('tar.gz'):
                        print("unpacking {}".format(file))
                        tar = tarfile.open(file, 'r:gz')
                        tar.extractall()
                        tar.close()
            else:
                print("image_data already exist")
