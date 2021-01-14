import rospy
from nav_msgs.msg import Odometry
from PyQt5.QtGui import QBrush, QPen
from PyQt5.QtCore import  Qt, pyqtSignal, pyqtSlot, QObject

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
        self.currentRegion = None
        self.regionLines = None
        # Subscribe
        self.sub = rospy.Subscriber(self.topic, Odometry, callback=self.callback)

    def targetCallback(self, msg):
        self.currentTarget = msg

    def regionCallback(self, msg):
        self.currentRegion = msg

    def callback(self, msg):
        """ Called when new message arrives in /odom """
        # Debug: print few lines only
        # rospy.loginfo("odom callback")
        if self.count < 0:
            return

        self.coords_in['x'] = msg.pose.pose.position.x
        self.coords_in['y'] = msg.pose.pose.position.y
        self.update_ui()
        # print str(self.name) + " previous: " + str(previous_coords) + " next: " + str(self.coords_in)


        #if self.coords_in == previous_coords:
    #        return
        #else:
            #self.update_ui()

    def update_ui(self):
        scene_coords = transformation.world_to_scene(self.coords_in.get('x'), self.coords_in.get('y'))
        #self.circle_draw.emit(self.id, scene_coords.get('x'), scene_coords.get('y'), self.RBT_DIAM)


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

        self.drawRegion()

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
        # Unsub when destroyedw
        self.sub.unregister()
