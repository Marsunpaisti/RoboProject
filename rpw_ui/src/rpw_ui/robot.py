import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Quaternion
from PyQt5.QtGui import QBrush, QPen, QColor, QFont
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QObject
from PyQt5.QtWidgets import QGraphicsDropShadowEffect
import math

import transformation
from tf.transformations import euler_from_quaternion


class Robot:
    ROBOT_DIAM = 0.35 # Robot graphical size in pixels

    def __init__(self, name, graphicsScene):
        self.name = name
        self.topic = "/{}/odom".format(name)
        rospy.loginfo("{} created and listens to topic: {}".format(self.name, self.topic))

        self.currentCoordinates = None
        self.rotation = None
        self.graphicsScene = graphicsScene
        self.locationCircle = None
        self.text = None
        self.cut_angle = 0
        self.increasing = True
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
        self.locationCircle.setPos(self.currentCoordinates.x, self.currentCoordinates.y)
        self.locationCircle.setRotation(self.rotation)
        self.advanceAngle()
        self.locationCircle.setStartAngle(self.cut_angle)
        self.locationCircle.setSpanAngle(5760 - 2 * self.cut_angle)


    def addItems(self):
        self.locationCircle = self.graphicsScene.addEllipse(-(Robot.ROBOT_DIAM/2.0), -(Robot.ROBOT_DIAM/2.0), Robot.ROBOT_DIAM,
                                                            Robot.ROBOT_DIAM, QPen(Qt.black, 0.01),
                                                            QBrush(Qt.red))

        centerCircle = self.graphicsScene.addEllipse(-0.05, -0.05, 0.10, 0.10, QPen(Qt.black, 0.01), QBrush(Qt.green))
        centerCircle.setParentItem(self.locationCircle)

        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(15)
        shadow.setOffset(0, 0)
        shadow.setColor(QColor(0, 0, 255))
        self.locationCircle.setGraphicsEffect(shadow)
        #font = QFont()
        #font.setPixelSize(0.12)
        #self.text = self.graphicsScene.addText(str(self.name), font)
        #self.text.setParentItem(self.locationCircle)

    def advanceAngle(self):
        max_angle, min_angle, step = 500, 0, 20

        if self.cut_angle < max_angle and self.increasing:
            self.cut_angle += step
            if self.cut_angle == max_angle:
                self.increasing = False
        else:
            self.cut_angle -= step
            if self.cut_angle == min_angle and not self.increasing:
                self.increasing = True


    def __del__(self):
        # Unsub when destroyed
        self.sub.unregister()