import os
import rospy
import rospkg
import re
from robot import Robot

from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PyQt5 import Qt, QtGui
from PyQt5.QtCore import  Qt, QTimer, pyqtSignal, pyqtSlot, QObject
from PyQt5.QtGui import QBrush, QPen
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QGraphicsView, QGraphicsItem, QGraphicsRectItem


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

        # Add scene, view,
        self.scene = QGraphicsScene(0,0,998,998) # setSceneRect as parameter
        # ROI rectangle
        self.roi_widht = 200
        self.roi_height = 200
        self.roi = self.scene.addRect(0, 0,self.roi_widht, self.roi_height, QPen(Qt.black), QBrush(Qt.gray))
        self.roi.setFlag(QGraphicsItem.ItemIsMovable, True)
        # Setup view
        view = QGraphicsView(self.scene)
        view.setGeometry(0,1000,0,1000)
        view.setFixedSize(1000, 1000)
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
        if rospy.has_param('robot_names_set'):
            param_str = rospy.get_param('robot_names_set')
            params = param_str.split()
            rospy.loginfo("Robot parameters: " + str(params))
            # Init gui robots
            for robot_id in params:
                self.robots.append( Robot(robot_id) )

            # Connect graphics signals
            for robot in self.robots:
                robot.circle_draw.connect(self.draw_circle)

        # Testing
        # self.scene.addEllipse(0,0,10,10, QPen(Qt.red), QBrush(Qt.red))
        # self.scene.addLine(0,0,100,100, QPen(Qt.green))
        self.roi.setPos(450,400)

    def start_button_clicked(self):
        if self._widget.start_button.text() == "Stop":
            rospy.loginfo("Stopping..")
            self.timer.stop()
            self._widget.start_button.setText("Start")
        else:
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
        # print(str_pos)

        # Calculate corner points
        roi_points = Polygon()
        half_width = float(self.roi_widht) / 2
        half_height = float(self.roi_height) / 2
        scale = 100
        # _l, _r = left / right; _u, _d = up / down
        x_l = (x - half_width) / scale
        y_u = (-y + half_height) / scale
        x_r = (x + half_width) / scale
        y_d = (-y - half_height) / scale
        rospy.loginfo("Sending to topic")
        z = 0.0
        roi_points.points.append(Point32(x_l, y_u, z))
        roi_points.points.append(Point32(x_r, y_u, z))
        roi_points.points.append(Point32(x_r, y_d, z))
        roi_points.points.append(Point32(x_l, y_d, z))

        self.pub.publish(roi_points)

    @pyqtSlot(int, int, int, int)
    def draw_circle(self, id, x, y, diam):
        self.robot_graphics.append( self.scene.addEllipse(x, y, diam, diam, QPen(Qt.black), QBrush(Qt.red)) )

    @pyqtSlot(int, int, int, int)
    def update_circle(self, id, x, y, diam):
        circle_to_move = self.robot_graphics[id+1]
        circle_to_move.setPox(x, y)

    def reset_button_clicked(self):
        self.roi.setPos(0,0)

    def shutdown_plugin(self):
        rospy.loginfo("Exiting..")
        self.pub.unregister()
        self.timer.stop()
        # Unsubscribe robots listening odom
        del self.robots
