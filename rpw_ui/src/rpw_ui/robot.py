import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Quaternion
from PyQt5.QtGui import QBrush, QPen
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QObject
from PyQt5.QtWidgets import QGraphicsDropShadowEffect
import math

import transformation
from tf.transformations import euler_from_quaternion


class Robot:
    ROBOT_DIAM = 30  # Robot graphical size in pixels

    def __init__(self, name, graphicsScene):
        self.name = name
        self.topic = "/{}/odom".format(name)
        rospy.loginfo("{} created and listens to topic: {}".format(self.name, self.topic))

        self.currentCoordinates = None
        self.rotation = None
        self.graphicsScene = graphicsScene
        self.locationCircle = None
        # Subscribe
        self.positionSubscriber = rospy.Subscriber(self.topic, Odometry, callback=self.callback)


    def callback(self, msg):
        """ Called when new message arrives in this robots /odom topic """
        self.currentCoordinates = Pose2D()
        self.currentCoordinates.x = msg.pose.pose.position.x
        self.currentCoordinates.y = msg.pose.pose.position.y

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

        euler = euler_from_quaternion(quaternion, axes='sxyz')
        self.rotation = - euler[2]
        self.rotation = math.degrees(self.rotation)

    def drawOnScene(self):
        if self.currentCoordinates is None:
            return

        drawCoords = transformation.world_to_scene(self.currentCoordinates.x, self.currentCoordinates.y)
        if self.locationCircle is None:
            self.addItems()
            self.locationCircle.setPos(drawCoords.get("x"), drawCoords.get("y"))
        else:

            self.locationCircle.setPos(drawCoords.get("x"), drawCoords.get("y"))
            #print str(self.rotation)
            self.locationCircle.setRotation(self.rotation)
    def addItems(self):
        self.locationCircle = self.graphicsScene.addEllipse(0, 0, Robot.ROBOT_DIAM,
                                                            Robot.ROBOT_DIAM, QPen(Qt.black),
                                                            QBrush(Qt.red))
        cut_angle = 500
        self.locationCircle.setStartAngle(cut_angle)
        self.locationCircle.setSpanAngle(5760 - 2 * cut_angle)

        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(10)
        self.locationCircle.setGraphicsEffect(shadow)

        text = self.graphicsScene.addText(str(self.name[4]))
        text.setParentItem(self.locationCircle)

    def __del__(self):
        # Unsub when destroyed
        self.sub.unregister()