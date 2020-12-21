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
        self.currentTarget = None
        self.rotation = None
        self.graphicsScene = graphicsScene
        self.locationCircle = None
        self.targetLine = None
        self.text = None
        self.cut_angle = 0
        self.increasing = True
        self.isSelected = False
        self.glowEffect = None
        # Subscribe
        self.positionSubscriber = rospy.Subscriber(self.topic, Odometry, callback=self.callback, queue_size=1)
        self.targetSubscriber = rospy.Subscriber("/{}/controller_target".format(self.name), Pose2D, callback=self.targetCallback)
        self.commandPublisher = rospy.Publisher("/{}/controller_target".format(self.name), Pose2D, queue_size=1)

    def targetCallback(self, msg):
        self.currentTarget = msg

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
        self.rotation = euler[2]
        self.rotation = math.degrees(self.rotation)

    def drawTargetLine(self):
        if (self.currentTarget == None):
            return

        if (self.targetLine == None):
            self.targetLine = self.graphicsScene.addLine(self.currentCoordinates.x, self.currentCoordinates.y, self.currentTarget.x, self.currentTarget.y, QPen(QColor(255, 0, 255), 0.01))
        else:
            self.targetLine.setLine(self.currentCoordinates.x, self.currentCoordinates.y, self.currentTarget.x, self.currentTarget.y)

    def drawOnScene(self):
        if self.currentCoordinates is None:
            return
        if self.locationCircle is None:
            self.addItems()
        self.glowEffect.setEnabled(self.isSelected)
        self.locationCircle.setPos(self.currentCoordinates.x, self.currentCoordinates.y)
        self.locationCircle.setRotation(self.rotation)
        self.advanceAngle()
        self.locationCircle.setStartAngle(self.cut_angle)
        self.locationCircle.setSpanAngle(5760 - 2 * self.cut_angle)

        self.drawTargetLine()


    def addItems(self):
        self.locationCircle = self.graphicsScene.addEllipse(-(Robot.ROBOT_DIAM/2.0), -(Robot.ROBOT_DIAM/2.0), Robot.ROBOT_DIAM,
                                                            Robot.ROBOT_DIAM, QPen(Qt.black, 0.01),
                                                            QBrush(Qt.red))
        self.glowEffect = QGraphicsDropShadowEffect()
        self.glowEffect.setBlurRadius(15)
        self.glowEffect.setOffset(0, 0)
        self.glowEffect.setColor(QColor(0, 0, 255))
        self.locationCircle.setGraphicsEffect(self.glowEffect)
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
        self.positionSubscriber.unregister()
        self.commandPublisher.unregister()