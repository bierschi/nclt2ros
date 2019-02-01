import os
import simplekml
from definitions import ROOT_DIR
from src.extractor.read import ReadRawData


class Plotter:

    def __init__(self, date):

        if isinstance(date, str):
            self.reader = ReadRawData(date=date)
        else:
            raise TypeError('"date" must be of type string')

        self.visualization_kml_dir = ROOT_DIR + '/plots/kml/'
        self.visualization_png_gt_dir = ROOT_DIR + '/plots/png/gt/'

        if not os.path.exists(self.visualization_kml_dir):
            os.makedirs(self.visualization_kml_dir)

        if not os.path.exists(self.visualization_png_gt_dir):
            os.makedirs(self.visualization_png_gt_dir)

        self.kml = simplekml.Kml()
        self.green = simplekml.Color.lime