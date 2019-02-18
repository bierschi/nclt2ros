import math
import numpy as np


class CoordinateFrame:
    """Class to transform between the odom and ground truth coordinate frame

    USAGE:
            CoordinateFrame(origin='odom')
            CoordinateFrame(origin='gt')
    """
    def __init__(self, origin='odom'):

        if isinstance(origin, str):
            if origin is 'gt':
                self.lat_origin = 42.293227       # degree
                self.lon_origin = -83.709657      # degree
            if origin is 'odom':
                self.lat_origin = 42.2939096617   # degree
                self.lon_origin = -83.7083507601  # degree
        else:
            raise TypeError("origin must be type of str")

        self.alt_origin         = 270      # meter
        self.earth_equat_radius = 6378135  # meter
        self.earth_polar_radius = 6356750  # meter

    def calc_radius_ns(self):
        """calculates the approximation of the earths radius in north-south direction

        :return: radius_ns: value of radius in meter
        """

        numerator = math.pow(self.earth_equat_radius*self.earth_polar_radius, 2)
        val1 = math.pow(self.earth_equat_radius * math.cos(np.deg2rad(self.lat_origin)), 2)
        val2 = math.pow(self.earth_polar_radius * math.sin(np.deg2rad(self.lat_origin)), 2)
        denominator = math.pow(val1 + val2, 1.5)
        radius_ns = numerator / denominator

        return radius_ns

    def calc_radius_ew(self):
        """calculates the approximation of the earths radius in east-west direction

        :return: radius_ew: value of radius in meter
        """
        numerator = math.pow(self.earth_equat_radius, 2)
        val1 = math.pow(self.earth_equat_radius * math.cos(np.deg2rad(self.lat_origin)), 2)
        val2 = math.pow(self.earth_polar_radius * math.sin(np.deg2rad(self.lat_origin)), 2)
        denominator = math.sqrt(val1 + val2)
        radius_ew = numerator / denominator

        return radius_ew

    def get_lat(self, x):
        """get the latitude value from x coordinate

        :param x: x coordinate in meters
        :return: lat: latitude in degree
        """

        tmp = x / self.calc_radius_ns()
        lat = np.rad2deg(math.asin(tmp)) + self.lat_origin

        return lat

    def get_lon(self, y):
        """get the longitude value from y coordinate

        :param y: y coordinate in meters
        :return: lon: longitude in degree
        """

        tmp = y / (self.calc_radius_ew() * math.cos(np.deg2rad(self.lat_origin)))
        lon = np.rad2deg(math.asin(tmp)) + self.lon_origin

        return lon

    def get_alt(self, z):
        """get the altitude value from z coordinate

        :param z: z coordinate in meters
        :return: altitude in meters
        """
        return self.alt_origin - z

    def get_x(self, lat):
        """get the x coordinate from the latitude

        :param lat: latitude in degree
        :return: x coordinate in meters
        """

        return math.sin(np.deg2rad(lat-self.lat_origin)) * self.calc_radius_ns()

    def get_y(self, lon):
        """get the y coordinate from the longitude

        :param lon: longitude in degree
        :return: y coordinate in meters
        """

        return math.sin(np.deg2rad(lon - self.lon_origin)) * self.calc_radius_ew() * math.cos(np.deg2rad(self.lat_origin))

    def get_z(self, alt):
        """get the z coordinate from the altitude

        :param alt: altitude in meters
        :return: z coordinate in meters
        """

        return self.alt_origin - alt
