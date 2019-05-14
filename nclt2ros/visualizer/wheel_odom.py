import math
import matplotlib.pyplot as plt
from nclt2ros.visualizer.plotter import Plotter
from nclt2ros.transformer.coordinate_frame import CoordinateFrame


class WheelOdom(Plotter):
    """Class to visualize the wheel odometry data as a kml and png file

    USAGE:
            WheelOdom(date='2013-01-10', output_file='wheel_odom', plt_show=True)

    """
    def __init__(self, date, output_file='wheel_odom', plt_show=True):

        if isinstance(output_file, str):
            self.output_file = output_file
        else:
            raise TypeError("'output_file' must be type of string")

        self.date = date
        self.plt_show = plt_show

        # init base class
        Plotter.__init__(self, date=self.date)

        self.offset_dates_odom = ['2012-01-08']

        # transformer coordinate frame into 'odom' or 'gt'
        if self.date in self.offset_dates_odom:
            self.odom_converter = CoordinateFrame(origin='gt')
        else:
            self.odom_converter = CoordinateFrame(origin='odom')

        # load data
        self.wheel_odom = self.reader.read_odometry_mu_100hz_csv(all_in_one=True)

    def save_kml_line(self):
        """visualize wheel odometry data as a kml file
        """

        wheel_odom_x = self.wheel_odom[:, 1]
        wheel_odom_y = self.wheel_odom[:, 2]
        odom_list = list()

        if len(wheel_odom_x) > 200000:
            divider = 15
        else:
            divider = 5

        for row, (x_i, y_j) in enumerate(zip(wheel_odom_x, wheel_odom_y)):

            if not math.isnan(x_i) and not math.isnan(y_j):
                lat = self.odom_converter.get_lat(x=x_i)
                lng = self.odom_converter.get_lon(y=y_j)
                tup = (lng, lat)

                # minimize the elements in the kml output file
                if (row % divider) == 0:
                    odom_list.append(tup)

        ls = self.kml.newlinestring(name="wheel odometry", coords=odom_list, description="latitute and longitude from wheel odometry")
        ls.style.linestyle.width = 1
        ls.style.linestyle.color = self.magenta

        self.kml.save(self.visualization_kml_dir + self.output_file + '.kml')

    def get_wheel_odom_data(self):
        """get wheel odometry data for visualization

        :return: list for x coordinates, list for y coordinates
        """
        wheel_odom_x = self.wheel_odom[:, 1]
        wheel_odom_y = self.wheel_odom[:, 2]

        return wheel_odom_x, wheel_odom_y

    def save_wheel_odom_png(self):
        """visualize wheel odometry data as a png file
        """
        wheel_odom_x, wheel_odom_y = self.get_wheel_odom_data()

        plt.plot(wheel_odom_y, wheel_odom_x, 'm-', label="wheel odom")
        plt.title('Wheel Odometry')
        plt.xlabel('x in meter')
        plt.ylabel('y in meter')
        plt.legend(loc='upper left')

        plt.grid()
        plt.savefig(self.visualization_png_wheel_odom_dir + 'wheel_odom.png')

        if self.plt_show:
            plt.show()

    def get_png_odom_dir(self):
        """get the png odom directory

        :return: path to png odom directory
        """
        return self.visualization_png_wheel_odom_dir


if __name__ == '__main__':
    odom = WheelOdom(date='2012-02-02')
    odom.save_wheel_odom_png()
    odom.save_kml_line()