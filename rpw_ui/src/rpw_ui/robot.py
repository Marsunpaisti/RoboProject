import rospy
from geometry_msgs.msg import Twist
from PyQt5.QtGui import QBrush, QPen
from PyQt5.QtCore import  Qt

class Robot:

    def __init__(self, id, scene):
        self.id = id
        self.topic = "/{}/odom".format(id)
        rospy.loginfo("Robot {} created and listens to topic: {}".format(self.id, self.topic))
        self.x_pos = 0.0
        self.y_pos = 0.0
        # Subscribe
        self.sub = rospy.Subscriber(self.topic, Twist, callback=self.callback)

        scene.addEllipse(0, 0, 30, 30, QPen(Qt.red), QBrush(Qt.red))

    def callback(self, msg):
        self.x_pos = msg.linear.x
        self.y_pos = msg.linear.y


    def __del__(self):
        # Unsub when destroyedw
        self.sub.unregister()