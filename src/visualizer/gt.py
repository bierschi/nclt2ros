import matplotlib.pyplot as plt
from src.visualizer.plotter import Plotter
from src.transform.coordinate_frame import CoordinateFrame


class GroundTruth(Plotter):
    """Class to visualize the ground truth as a kml and png file

    USAGE:
            GroundTruth(date='2013-01-10', output_file='ground_truth')

    """
    def __init__(self, date, output_file):

        if isinstance(output_file, str):
            self.output_file = output_file
        else:
            raise TypeError("'output_file' must be type of string")

        # init base class
        Plotter.__init__(self, date=date)

        # transform coordinate frame into 'gt'
        self.gt_converter = CoordinateFrame(origin='gt')

        # load gt data
        self.utimes, self.x, self.y, self.z, self.roll_rad, self.pitch_rad, self.yaw_rad = self.reader.read_gt_csv()

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

    def save_png(self, offset=True):
        """visualize the ground truth as a png file

        :param offset: Boolean, True if eliminate the offset between odom and ground truth coordinate frame
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

        plt.plot(y_new, x_new, color="lime", label='ground truth')

        plt.xlabel('x in meter')
        plt.ylabel('y in meter')
        plt.legend(loc='upper left')

        plt.grid()

        if offset is True:
            plt.title('Ground Truth Offset')
            plt.savefig(self.visualization_png_gt_dir + self.output_file + 'offset.png')
        else:
            plt.title('Ground Truth')
            plt.savefig(self.visualization_png_gt_dir + self.output_file + '.png')

        plt.show()


if __name__ == '__main__':
    gt = GroundTruth('2013-01-10', output_file='ground_truth')
    #gt.save_kml_line()
    gt.save_png(offset=True)
