import numpy as np
import matplotlib.pyplot as plt
from src.visualizer.plotter import Plotter
from src.transformer.coordinate_frame import CoordinateFrame


class GPS_RTK(Plotter):
    """Class to visualize the GPS RTK as a kml and png file

    USAGE:
            GPS_RTK(date='2013-01-10', output_file='gps_rtk')

    """
    def __init__(self, date, output_file='gps_rtk'):

        if isinstance(output_file, str):
            self.output_file = output_file
        else:
            raise TypeError("'output_file' must be type of string")

        # init base class
        Plotter.__init__(self, date=date)

        # transformer coordinate frame into 'odom'
        self.gps_rtk_converter = CoordinateFrame(origin='odom')

        # load data
        self.gps_rtk = self.reader.read_gps_rtk_csv(all_in_one=True)

    def save_kml_line(self):
        """visualize the gps rtk data as a kml file

        """

        lat = self.gps_rtk[:, 3]
        lon = self.gps_rtk[:, 4]
        gps_rtk_list = list()

        for (i_lat, j_lon) in zip(lat, lon):
            tup = (np.rad2deg(j_lon), np.rad2deg(i_lat))  # swap and convert lat long to deg
            gps_rtk_list.append(tup)

        ls = self.kml.newlinestring(name="gps rtk", coords=gps_rtk_list, description="latitude, longitude from gps rtk")
        ls.style.linestyle.width = 1
        ls.style.linestyle.color = self.red

        self.kml.save(self.visualization_kml_dir + self.output_file + '.kml')

    def get_gps_rtk_data(self):
        """get gps rtk data for visualization

        :return: list for x coordinates, list for y coordinates
        """
        lat = self.gps_rtk[:, 3]
        lon = self.gps_rtk[:, 4]
        x_list = list()
        y_list = list()

        for (i_lat, j_lon) in zip(lat, lon):
            x = self.gps_rtk_converter.get_x(lat=np.rad2deg(i_lat))
            y = self.gps_rtk_converter.get_y(lon=np.rad2deg(j_lon))
            x_list.append(x)
            y_list.append(y)

        return x_list, y_list

    def save_gps_rtk_png(self):
        """visualize the gps rtk data as a png file

        """

        x_list, y_list = self.get_gps_rtk_data()

        plt.plot(y_list, x_list, 'r-', label='gps rtk')
        plt.title('GPS RTK')
        plt.xlabel('x in meter')
        plt.ylabel('y in meter')
        plt.legend(loc='upper left')

        plt.grid()
        plt.savefig(self.visualization_png_gps_rtk_dir + 'gps_rtk.png')

        plt.show()


if __name__ == '__main__':
    gps = GPS_RTK('2013-01-10')
    #gps.save_kml_line()
    gps.save_gps_rtk_png()
