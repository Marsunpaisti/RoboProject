import rospy
from geometry_msgs.msg import Twist
from PyQt5.QtGui import QBrush, QPen
from PyQt5.QtCore import  Qt, pyqtSignal, pyqtSlot, QObject

import transformation

class Robot(QObject):

    def __init__(self, id):
        QObject.__init__(self)
        self.id = id
        self.topic = "/{}/odom".format(id)
        rospy.loginfo("{} created and listens to topic: {}".format(self.id, self.topic))

        self.coords_in = dict(x=0.0, y=0.0)
        self.circle = None
        print self.coords_in
        # Subscribe
        self.sub = rospy.Subscriber(self.topic, Twist, callback=self.callback)

        self.count = 15

    def callback(self, msg):
        if self.count < 0:
            return
        self.count = self.count - 1
        previous_coords = self.coords_in.copy()

        self.coords_in['x'] = msg.linear.x
        self.coords_in['y'] = msg.linear.y
        # print str(self.id) + " previous: " + str(previous_coords) + " next: " + str(self.coords_in)

        if self.coords_in == previous_coords:
            return
        else:
            self.update_ui()

    def update_ui(self):

        if not self.circle:
            self.circle = "yes"
            print "draw circle"
            # self.scene.addEllipse(0, 0, 30, 30, QPen(Qt.red), QBrush(Qt.red))

        pass

    def __del__(self):
        # Unsub when destroyedw
        self.sub.unregister()