import os
import rospy
import rospkg
import re
from robot import Robot
import transformation
from geometry_msgs.msg import Polygon, Point, Pose2D
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PyQt5 import Qt, QtGui
from PyQt5.QtCore import  Qt, QTimer, pyqtSignal, pyqtSlot, QObject
from PyQt5.QtGui import QBrush, QPen, QTransform, QColor, QFont
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QGraphicsView, QGraphicsItem, QGraphicsRectItem

screenSize = (1000, 1000)  # Screen widht px, Screen height px

class InteractableScene(QGraphicsScene):
    def __init__(self, x, y, w, h):
        super(InteractableScene, self).__init__(x, y, w, h)
        self.clickHandler = None
        self.keyPressHandler = None

    def mousePressEvent(self, event):
        super(InteractableScene, self).mousePressEvent(event)
        clickedItem = self.itemAt(event.scenePos(), QTransform())
        if (self.clickHandler != None):
            self.clickHandler(event, clickedItem)

    def keyPressEvent(self, event):
        super(InteractableScene, self).keyPressEvent(event)
        #rospy.loginfo("Keypress: {}".format(event.key()))
        if (self.keyPressHandler != None):
            self.keyPressHandler(event)
        

class MyPlugin(Plugin):
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this packagehttps://doc.qt.io/qt-5/qtmodules.html
        ui_file = os.path.join(rospkg.RosPack().get_path('rpw_ui'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Fix button area
        self._widget.setFixedWidth(270)
        # Add .ui file's widget to the user interface
        context.add_widget(self._widget)

        # Add scene, view
        self.scene = InteractableScene(0, 0, screenSize[0], screenSize[1]) # setSceneRect as parameter
        self.scene.clickHandler = self.clickHandler
        self.scene.keyPressHandler = self.keyPressHandler
        # Coordinate axes
        self.scene.addLine(0, 0, 100, 0, QPen(Qt.red, 0.01))
        self.scene.addLine(0, 0, 0, 100, QPen(Qt.green, 0.01))

        # ROI rectangle
        self.roi_width = 3
        self.roi_height = 3
        self.roi = self.scene.addRect(-self.roi_width/2.0, -self.roi_height/2.0 ,self.roi_width, self.roi_height, QPen(Qt.black, 0.01), QBrush(QColor(0, 0, 0, 50)))
        self.roi.setFlag(QGraphicsItem.ItemIsMovable, True)

        # Setup view
        view = QGraphicsView(self.scene)
        border_left = 0
        border_right = screenSize[0]
        border_up = 0
        border_down = screenSize[1]
        view.setGeometry(border_left, border_right, border_up, border_down)
        view.setFixedSize(*screenSize)
        view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        viewMatrix = QTransform()
        viewMatrix.scale(100, -100)
        view.setTransform(viewMatrix)
        view.setSceneRect(-4.9, -4.9, 9.8, 9.8)
        view.centerOn(0, 0)

        context.add_widget(view)

        # Publisher timer and start / stop / reset button
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_roi_position)
        self._widget.start_button.clicked.connect(self.start_button_clicked)
        self._widget.reset_button.clicked.connect(self.reset_button_clicked)


        # Publisher
        self.pub = rospy.Publisher('target_region', Polygon , queue_size=1)

        # Get robot parameters
        self.robots = []
        self.selectedRobot = None
        self.robot_graphics = []
        param_str = ""
        try:
            param_str = rospy.get_param('robot_names_set')
        except:
            rospy.loginfo("Robot names unavailable, defaulting to robo1 robo2 robo3")
            param_str = "robo1 robo2 robo3"

        params = param_str.split()
        rospy.loginfo("Robot parameters: " + str(params))
        # Init gui robots
        for robot_id in params:
            self.robots.append( Robot(robot_id, self.scene) )

        self.graphics_timer = QTimer()
        self.graphics_timer.timeout.connect(self.drawRobots)
        self.graphics_timer.start(15)

    def start_button_clicked(self):
        if self._widget.start_button.text() == "Stop":
            rospy.loginfo("Stopping publishing to /target_region")
            self.timer.stop()
            self._widget.start_button.setText("Start")
        else:
            rospy.loginfo("Starting publishing to /target_region")
            self._widget.start_button.setText("Stop")
            interval = self._widget.interval_spinbox.value()
            self.timer.start(interval)

    def keyPressHandler(self, event):
        x = self.roi.pos().x()
        y = self.roi.pos().y()
        width = self.roi.rect().width()
        height = self.roi.rect().height()
        if (event.key() == Qt.Key_Up):
            self.roi.setRect(-width/2.0 - 0.05, -height/2.0 - 0.05, width + 0.1, height + 0.1)
            #rospy.loginfo(str(-width/2.0) +" "+ str(-height/2.0) +" "+ str(width + 0.1) +" "+ str(height + 0.1))
        elif (event.key() == Qt.Key_Down):
            self.roi.setRect(-width/2.0 + 0.05, -height/2.0 + 0.05, width - 0.1, height - 0.1)
            #rospy.loginfo(str(-width / 2.0) +" "+ str(-height / 2.0) +" "+ str(width - 0.1) +" "+ str(height - 0.1))

    def update_roi_position(self):
        # Get widget's position
        x = float(self.roi.pos().x())
        y = float(self.roi.pos().y())
        width = self.roi.rect().width()
        height = self.roi.rect().height()
        # Update GUI's position indicator
        str_pos = str(x) + ", " + str(y)
        self._widget.position_edit.setText(str_pos)

        # Calculate ROI corner points
        roi_points = Polygon()
        x_left = x - width/2.0
        y_up = y - width/2.0
        x_right = x + width/2.0
        y_down = y + height/2.0

        coords = [(x_left, y_up),
                  (x_right, y_up),
                  (x_right, y_down),
                  (x_left, y_down)]

        z = 0.0
        for coord in coords:
            roi_points.points.append(Point(coord[0], coord[1], z ))

        self.pub.publish(roi_points)

    @pyqtSlot()
    def drawRobots(self):
        for robot in self.robots:
            robot.drawOnScene()

    def reset_button_clicked(self):
        self.roi.setPos(0,0)

    def clickHandler(self, event, clickedItem):
        rospy.loginfo("Pressed x {} y {} {}".format(event.scenePos().x(), event.scenePos().y(), clickedItem))
        if (event.button() == 1):
            self.selectedRobot = None
            for robot in self.robots:
                robot.isSelected = False
                if (robot.locationCircle == clickedItem):
                    # rospy.loginfo("Selected robot {}".format(robot.name))
                    self.selectedRobot = robot
                    robot.isSelected = True
        elif (event.button() == 2 and self.selectedRobot != None):
            rospy.loginfo("Assigned target to robot {}".format(self.selectedRobot.name))
            targetPose = Pose2D()
            targetPose.x = event.scenePos().x()
            targetPose.y = event.scenePos().y()
            self.selectedRobot.commandPublisher.publish(targetPose)
            self.selectedRobot.isSelected = False
            self.selectedRobot = None


    def shutdown_plugin(self):
        rospy.loginfo("Exiting")
        self.pub.unregister()
        self.timer.stop()
        # Unsubscribe robots listening odom
        del self.robots
