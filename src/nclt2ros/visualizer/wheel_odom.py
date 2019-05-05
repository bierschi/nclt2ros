import matplotlib.pyplot as plt
from src.nclt2ros.visualizer.plotter import Plotter
from src.nclt2ros import CoordinateFrame


class WheelOdom(Plotter):
    """Class to visualize the wheel odometry data as a kml and png file

    USAGE:
            WheelOdom(date='2013-01-10', output_file='wheel_odom')

    """
    def __init__(self, date, output_file='wheel_odom'):

        if isinstance(output_file, str):
            self.output_file = output_file
        else:
            raise TypeError("'output_file' must be type of string")

        # init base class
        Plotter.__init__(self, date=date)

        # transformer coordinate frame into 'odom'
        self.odom_converter = CoordinateFrame(origin='odom')

        # load data
        self.wheel_odom = self.reader.read_odometry_mu_100hz_csv(all_in_one=True)

    def save_kml_line(self):
        """visualize wheel odometry data as a kml file
        """

        wheel_odom_x = self.wheel_odom[:, 1]
        wheel_odom_y = self.wheel_odom[:, 2]
        odom_list = list()

        for row, (x_i, y_j) in enumerate(zip(wheel_odom_x, wheel_odom_y)):

            lat = self.odom_converter.get_lat(x=x_i)
            lng = self.odom_converter.get_lon(y=y_j)
            tup = (lng, lat)

            # minimize the elements in the kml output file
            if (row % 5) == 0:
                odom_list.append(tup)

        ls = self.kml.newlinestring(name="wheel odometry", coords=odom_list, description="latitute, longitude from wheel odometry")
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
        plt.legend()

        plt.grid()
        plt.savefig(self.visualization_png_wheel_odom_dir + 'wheel_odom.png')

        plt.show()

    def get_png_odom_dir(self):
        """get the png odom directory

        :return: path to png odom directory
        """
        return self.visualization_png_wheel_odom_dir

