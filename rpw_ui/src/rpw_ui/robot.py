import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from PyQt5.QtGui import QBrush, QPen
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QObject
import time

import transformation


class Robot:
    ROBOT_DIAMETER_PIXELS = 30  # Robot graphical size in pixels

    def __init__(self, name, graphicsScene):
        self.name = name
        self.topic = "/{}/odom".format(name)
        rospy.loginfo("{} created and listens to topic: {}".format(self.name, self.topic))

        self.currentCoordinates = None
        self.graphicsScene = graphicsScene
        self.locationCircle = None
        # Subscribe
        self.positionSubscriber = rospy.Subscriber(self.topic, Odometry, callback=self.callback)

    def callback(self, msg):
        """ Called when new message arrives in this robots /odom topic """
        self.currentCoordinates = Pose2D()
        self.currentCoordinates.x = msg.pose.pose.position.x
        self.currentCoordinates.y = msg.pose.pose.position.y

    def drawOnScene(self):
        if self.currentCoordinates is None:
            return

        drawCoords = transformation.world_to_scene(self.currentCoordinates.x, self.currentCoordinates.y)
        if self.locationCircle is None:
            self.locationCircle = self.graphicsScene.addEllipse(0, 0, Robot.ROBOT_DIAMETER_PIXELS,
                                                                Robot.ROBOT_DIAMETER_PIXELS, QPen(Qt.black),
                                                                QBrush(Qt.red))
            self.locationCircle.setPos(drawCoords.get("x"), drawCoords.get("y"))
        else:
            self.locationCircle.setPos(drawCoords.get("x"), drawCoords.get("y"))

    def __del__(self):
        # Unsub when destroyed
        self.sub.unregister()