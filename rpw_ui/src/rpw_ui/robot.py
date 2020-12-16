import rospy
from nav_msgs.msg import Odometry
from PyQt5.QtGui import QBrush, QPen
from PyQt5.QtCore import  Qt, pyqtSignal, pyqtSlot, QObject

import transformation

class Robot(QObject):

    circle_draw = pyqtSignal(int, int, int, int)  # ( robot id / x / y / diameter )
    circle_update = pyqtSignal(int, int, int, int) # ( robot id / x / y / diameter )

    def __init__(self, name):
        QObject.__init__(self)
        self.name = name                                # 'roboN'
        self.id = int( filter(str.isdigit, name) )      # N
        self.topic = "/{}/odom".format(name)
        rospy.loginfo("{} created and listens to topic: {}".format(self.name, self.topic))

        self.coords_in = dict(x=0.0, y=0.0)
        self._graphics_drawn = False

        # Subscribe
        self.sub = rospy.Subscriber(self.topic, Odometry, callback=self.callback)

        self.count = 15 # Debug

        self.RBT_DIAM = 30  # Robot graphical size in pixels

    def callback(self, msg):
        """ Called when new message arrives in /odom """
        previous_coords = self.coords_in.copy()

        precision = 2
        self.coords_in['x'] = round( msg.pose.pose.position.x, precision )
        self.coords_in['y'] = round( msg.pose.pose.position.y, precision )

        # print "prev coords: " + str(previous_coords) + ", coords in: " + str(self.coords_in)

        if self.coords_in == previous_coords:
            return
        else:
            self.update_ui()

    def update_ui(self):
        scene_coords = transformation.world_to_scene(self.coords_in.get('x'), self.coords_in.get('y'))

        if not self._graphics_drawn:
            self.circle_draw.emit(self.id, scene_coords.get('x'), scene_coords.get('y'), self.RBT_DIAM)
            self._graphics_drawn = True
        else:

            self.circle_update.emit(self.id, scene_coords.get('x'), scene_coords.get('y'), self.RBT_DIAM)


    def __del__(self):
        # Unsub when destroyedw
        self.sub.unregister()