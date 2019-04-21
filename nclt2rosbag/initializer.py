import rospy


class Initializer:

    def __init__(self):
        self.date = rospy.get_param('~date', '2013-01-10')
        print(self.date)
