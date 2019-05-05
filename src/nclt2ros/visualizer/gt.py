import matplotlib.pyplot as plt

from src.nclt2ros.visualizer.plotter import Plotter
from src.nclt2ros import CoordinateFrame


class GroundTruth(Plotter):
    """Class to visualize the ground truth data as a kml and png file

    USAGE:
            GroundTruth(date='2013-01-10', output_file='ground_truth')

    """
    def __init__(self, date, output_file='ground_truth'):

        if isinstance(output_file, str):
            self.output_file = output_file
        else:
            raise TypeError("'output_file' must be type of string")

        # init base class
        Plotter.__init__(self, date=date)

        # transformer coordinate frame into 'gt'
        self.gt_converter = CoordinateFrame(origin='gt')

        # load gt data
        self.utimes, self.x, self.y, self.z, self.roll_rad, self.pitch_rad, self.yaw_rad = self.reader_gt.read_gt_csv()

    def save_kml_line(self):
        """visualize the ground truth as a kml file
        """

        gt_list = list()

        for row, (x_i, y_j) in enumerate(zip(self.x, self.y)):

            lat = self.gt_converter.get_lat(x=x_i)
            lng = self.gt_converter.get_lon(y=y_j)

            tup = (lng, lat)

            # minimize the elements in the kml output file
            if (row % 3) == 0:
                gt_list.append(tup)

        # create line string
        ls = self.kml.newlinestring(name="ground truth", coords=gt_list, description="latitude, longitude from ground truth",)
        ls.style.linestyle.width = 1
        ls.style.linestyle.color = self.green

        # save kml file in visualization directory
        self.kml.save(self.visualization_kml_dir + self.output_file + '.kml')

    def get_gt_data(self, offset=True):
        """get ground truth data for visualization

        :return: list for x coordinates, list for y coordinates
        """
        x_new = list()
        y_new = list()

        for row, (x_i, y_j) in enumerate(zip(self.x, self.y)):

            if offset is True:
                x_i = x_i - 75.829339527800
                y_j = y_j - 107.724666286
                y_new.append(y_j)
                x_new.append(x_i)
            else:
                y_new.append(y_j)
                x_new.append(x_i)

        return x_new, y_new

    def save_gt_png(self, offset=True):
        """visualize the ground truth as a png file

        :param offset: Boolean, True if eliminate the offset between odom and ground truth coordinate frame
        """

        x_new, y_new = self.get_gt_data(offset=offset)

        plt.plot(y_new, x_new, color="lime", label='ground truth')

        plt.xlabel('x in meter')
        plt.ylabel('y in meter')
        plt.legend(loc='upper left')

        plt.grid()

        if offset is True:
            plt.title('Ground Truth Offset')
            plt.savefig(self.visualization_png_gt_dir + self.output_file + '_offset.png')
        else:
            plt.title('Ground Truth')
            plt.savefig(self.visualization_png_gt_dir + self.output_file + '.png')

        plt.show()

    def save_roll_png(self):
        """visualize the roll angle as a png file
        """

        plt.plot(self.utimes, self.roll_rad, color="blue", label='roll angle')

        plt.xlabel('time in sec')
        plt.ylabel('roll in rad')
        plt.legend(loc='upper left')
        plt.grid()

        plt.title('Roll angle from Ground Truth')
        plt.savefig(self.visualization_png_gt_dir + self.output_file + '_roll.png')
        plt.show()

    def save_pitch_png(self):
        """visualize the pitch angle as a png file
        """
        plt.plot(self.utimes, self.pitch_rad, color="blue", label='pitch angle')

        plt.xlabel('time in sec')
        plt.ylabel('pitch in rad')
        plt.legend(loc='upper left')
        plt.grid()

        plt.title('Pitch angle from Ground Truth')
        plt.savefig(self.visualization_png_gt_dir + self.output_file + '_pitch.png')
        plt.show()

    def save_yaw_png(self):
        """visualize the yaw angle as a png file
        """
        plt.plot(self.utimes, self.yaw_rad, color="blue", label='yaw angle')

        plt.xlabel('time in sec')
        plt.ylabel('yaw in rad')
        plt.legend(loc='upper left')
        plt.grid()

        plt.title('Yaw angle from Ground Truth')
        plt.savefig(self.visualization_png_gt_dir + self.output_file + '_yaw.png')
        plt.show()

    def get_png_gt_dir(self):
        """get the png ground truth directory

        :return: path to png ground truth directory
        """
        return self.visualization_png_gt_dir
