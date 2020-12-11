import rospy
from geometry_msgs.msg import Twist
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
        print self.coords_in
        # Subscribe
        self.sub = rospy.Subscriber(self.topic, Twist, callback=self.callback)

        self.count = 15 # Debug

        self.RBT_DIAM = 30  # Robot graphical size in pixels

    def callback(self, msg):
        """ Called when new message arrives in /odom """
        # Debug: print few lines only
        if self.count < 0:
            return
        self.count = self.count - 1
        # /Debug
        previous_coords = self.coords_in.copy()

        self.coords_in['x'] = msg.linear.x
        self.coords_in['y'] = msg.linear.y
        # print str(self.name) + " previous: " + str(previous_coords) + " next: " + str(self.coords_in)

        if self.coords_in == previous_coords:
            return
        else:
            self.update_ui()

    def update_ui(self):

        if not self._graphics_drawn:
            scene_coords = transformation.world_to_scene( self.coords_in.get('x'), self.coords_in.get('y') )
            self.circle_draw.emit(self.id, scene_coords.get('x'), scene_coords.get('y'), self.RBT_DIAM)
            print "draw circle emited"
            self._graphics_drawn = True

        pass

    def __del__(self):
        # Unsub when destroyedw
        self.sub.unregister()