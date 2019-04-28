import numpy as np
import matplotlib.pyplot as plt

from nclt2ros.visualizer.plotter import Plotter
from nclt2ros.transformer.coordinate_frame import CoordinateFrame


class GPS(Plotter):
    """Class to visualize the GPS as a kml and png file

    USAGE:
            GPS(date='2013-01-10', output_file='gps')

    """
    def __init__(self, date, output_file='gps'):
        if isinstance(output_file, str):
            self.output_file = output_file
        else:
            raise TypeError("'output_file' must be type of string")

        # init base class
        Plotter.__init__(self, date=date)

        # transformer coordinate frame into 'odom'
        self.gps_converter = CoordinateFrame(origin='odom')

        # load data
        self.gps     = self.reader.read_gps_csv(all_in_one=True)

    def save_kml_line(self):
        """visualize the gps data as a kml file

        """

        lat = self.gps[:, 3]
        lon = self.gps[:, 4]
        gps_list = list()

        for (i, j) in zip(lat, lon):
            tup = (np.rad2deg(j), np.rad2deg(i))  # swap and convert lat long to deg
            gps_list.append(tup)

        ls = self.kml.newlinestring(name="gps", coords=gps_list, description="latitude, longitude from gps")
        ls.style.linestyle.width = 1
        ls.style.linestyle.color = self.yellow

        self.kml.save(self.visualization_kml_dir + self.output_file + '.kml')

    def get_gps_data(self):
        """get gps data for visualization

        :return: list for x coordinates, list for y coordinates
        """
        lat = self.gps[:, 3]
        lon = self.gps[:, 4]
        x_list = list()
        y_list = list()

        for (i, j) in zip(lat, lon):
            x = self.gps_converter.get_x(lat=np.rad2deg(i))
            y = self.gps_converter.get_y(lon=np.rad2deg(j))
            x_list.append(x)
            y_list.append(y)

        return x_list, y_list

    def save_gps_png(self):
        """visualize the gps data as a kml file

        """

        x_list, y_list = self.get_gps_data()

        plt.plot(y_list, x_list, 'y-', label='gps')
        plt.title('GPS')
        plt.xlabel('x in meter')
        plt.ylabel('y in meter')
        plt.legend(loc='upper left')

        plt.grid()
        plt.savefig(self.visualization_png_gps_dir + 'gps.png')

        plt.show()

