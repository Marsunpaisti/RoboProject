import os
import rospy
import rospkg
import re
from robot import Robot
import transformation

from geometry_msgs.msg import Polygon, Point32


from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PyQt5 import Qt, QtGui
from PyQt5.QtCore import  Qt, QTimer, pyqtSignal, pyqtSlot, QObject
from PyQt5.QtGui import QBrush, QPen, QTransform, QColor, QFont
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QGraphicsView, QGraphicsItem, QGraphicsRectItem

screenSize = (1000, 1000)  # Screen widht px, Screen height px

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
        self.scene = QGraphicsScene(0, 0, screenSize[0] - 2, screenSize[1] - 2) # setSceneRect as parameter

        # Coordinate axes
        self.scene.addLine(0, 0, 100, 0, QPen(Qt.red, 0.01))
        self.scene.addLine(0, 0, 0, 100, QPen(Qt.green, 0.01))

        # ROI rectangle
        self.roi_widht = 3
        self.roi_height = 3
        self.roi = self.scene.addRect(0, 0,self.roi_widht, self.roi_height, QPen(Qt.black, 0.01), QBrush(QColor(0, 0, 0, 50)))
        self.roi.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.roi.setPos(-self.roi_widht/2.0, -self.roi_height/2.0)

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
        view.setSceneRect(-5, -5, 10, 10)
        view.centerOn(0, 0)
        #view.onClick.connect(self.onClick)

        context.add_widget(view)

        # Publisher timer and start / stop / reset button
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_roi_position)
        self._widget.start_button.clicked.connect(self.start_button_clicked)
        self._widget.reset_button.clicked.connect(self.reset_button_clicked)


        # Publisher
        self.pub = rospy.Publisher('target_region', Polygon , queue_size=10)

        # Get robot parameters
        self.robots = []
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

    def update_roi_position(self):
        # Get widget's position
        x = float(self.roi.pos().x())
        y = float(self.roi.pos().y())
        # Update GUI's position indicator
        str_pos = str(x) + ", " + str(y)
        self._widget.position_edit.setText(str_pos)

        # Calculate ROI corner points
        roi_points = Polygon()
        x_left = x
        y_up = y
        x_right = x + self.roi_widht
        y_down = y + self.roi_height

        coords = [(x_left, y_up),
                  (x_right, y_up),
                  (x_right, y_down),
                  (x_left, y_down)]

        z = 0.0
        for i in range(len(coords)):
            roi_points.points.append(Point32(coords[i][0], coords[i][1], z ))

        self.pub.publish(roi_points)


    @pyqtSlot()
    def drawRobots(self):
        for robot in self.robots:
            robot.drawOnScene()

    @pyqtSlot(float, float)
    def onClick(self, x, y):
        rospy.loginfo("Click on X: {} Y: {}".format(x, y))

    def reset_button_clicked(self):
        self.roi.setPos(0,0)

    def shutdown_plugin(self):
        rospy.loginfo("Exiting")
        self.pub.unregister()
        self.timer.stop()
        # Unsubscribe robots listening odom
        del self.robots
