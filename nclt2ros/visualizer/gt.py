import math
import matplotlib.pyplot as plt

from nclt2ros.visualizer.plotter import Plotter
from nclt2ros.transformer.coordinate_frame import CoordinateFrame


class GroundTruth(Plotter):
    """Class to visualize the ground truth data as a kml and png file

    USAGE:
            GroundTruth(date='2013-01-10', output_file='ground_truth', plt_show=True)

    """
    def __init__(self, date, output_file='ground_truth', plt_show=True):

        if isinstance(output_file, str):
            self.output_file = output_file
        else:
            raise TypeError("'output_file' must be type of string")

        self.date = date
        self.plt_show = plt_show

        # init base class
        Plotter.__init__(self, date=self.date)

        # transformer coordinate frame into 'gt'
        self.gt_converter = CoordinateFrame(origin='gt')

        # load gt data
        self.gt = self.reader_gt.read_gt_csv(all_in_one=True)

    def save_kml_line(self):
        """visualize the ground truth as a kml file
        """

        gt_x = self.gt[:, 1]
        gt_y = self.gt[:, 2]
        gt_list = list()

        if len(gt_x) > 600000:
            divider = 35
        elif len(gt_x) > 200000:
            divider = 15
        else:
            divider = 5

        for row, (x_i, y_j) in enumerate(zip(gt_x, gt_y)):

            if not math.isnan(x_i) and not math.isnan(y_j):
                lat = self.gt_converter.get_lat(x=x_i)
                lng = self.gt_converter.get_lon(y=y_j)

                tup = (lng, lat)

                # minimize the elements in the kml output file
                if (row % divider) == 0:
                    gt_list.append(tup)

        # create line string
        ls = self.kml.newlinestring(name="ground truth", coords=gt_list, description="latitude and longitude from ground truth",)
        ls.style.linestyle.width = 1
        ls.style.linestyle.color = self.green

        # save kml file in visualization directory
        self.kml.save(self.visualization_kml_dir + self.output_file + '.kml')

    def get_gt_data(self):
        """get ground truth data for visualization

        :return: list for x coordinates, list for y coordinates
        """
        gt_x = self.gt[:, 1]
        gt_y = self.gt[:, 2]
        first_x_coord = gt_x[0]
        first_y_coord = gt_y[0]
        x_new = list()
        y_new = list()

        for row, (x_i, y_j) in enumerate(zip(gt_x, gt_y)):

            if not math.isnan(x_i) and not math.isnan(y_j):
                # eliminate offset in this dataset
                if self.date == '2013-01-10':
                    x_i = x_i - first_x_coord
                    y_j = y_j - first_y_coord
                    y_new.append(y_j)
                    x_new.append(x_i)
                else:
                    y_new.append(y_j)
                    x_new.append(x_i)

        return x_new, y_new

    def save_gt_png(self):
        """visualize the ground truth as a png file

        :param offset: Boolean, True if eliminate the offset between odom and ground truth coordinate frame
        """

        x_new, y_new = self.get_gt_data()

        plt.plot(y_new, x_new, color="lime", label='ground truth')

        plt.xlabel('x in meter')
        plt.ylabel('y in meter')
        plt.legend(loc='upper left')

        plt.grid()

        plt.title('Ground Truth')
        plt.savefig(self.visualization_png_gt_dir + self.output_file + '.png')

        if self.plt_show:
            plt.show()

    def save_roll_png(self):
        """visualize the roll angle as a png file
        """

        utimes   = self.gt[:, 0]
        roll_rad = self.gt[:, 4]
        plt.plot(utimes, roll_rad, color="blue", label='roll angle')

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
        utimes    = self.gt[:, 0]
        pitch_rad = self.gt[:, 5]

        plt.plot(utimes, pitch_rad, color="blue", label='pitch angle')

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
        utimes   = self.gt[:, 0]
        yaw_rad  = self.gt[:, 6]

        plt.plot(utimes, yaw_rad, color="blue", label='yaw angle')

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


if __name__ == '__main__':
    gt = GroundTruth(date='2012-01-15')
    gt.save_gt_png()
