import rospy
from geometry_msgs.msg import Twist
from PyQt5.QtGui import QBrush, QPen
from PyQt5.QtCore import  Qt, pyqtSignal

import transformation

class Robot:

    def __init__(self, id, scene):
        self.id = id
        self.scene = scene
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
        previous_coords = self.coords_in
        print str(self.id) + "previous: " + str(previous_coords) + "next: " + str(self.coords_in)
        # self.x_pos = msg.linear.x
        # self.y_pos = msg.linear.y
        self.coords_in['x'] = msg.linear.x
        self.coords_in['y'] = msg.linear.y

        if cmp(self.coords_in, previous_coords):
            return
        else:
            self.update_ui()

    def update_ui(self):

        if not self.circle:
            self.scene.addEllipse(0, 0, 30, 30, QPen(Qt.red), QBrush(Qt.red))

        pass

    def __del__(self):
        # Unsub when destroyedw
        self.sub.unregister()