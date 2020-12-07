import os
import rospy
import rospkg

from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from std_msgs.msg import String

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PyQt5 import QtGui
from PyQt5.QtCore import  Qt, QTimer
from PyQt5.QtGui import QBrush, QPen
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QGraphicsView, QGraphicsItem


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

        # Add .ui file's widget to the user interface
        context.add_widget(self._widget)

        # Add scene, view, ROI rectangle
        scene = QGraphicsScene(0,0,1000,500)

        self.roi_widht = 200
        self.roi_height = 200
        self.roi = scene.addRect(0,0,self.roi_widht, self.roi_height, QPen(Qt.black), QBrush(Qt.gray))
        self.roi.setFlag(QGraphicsItem.ItemIsMovable, True)
        view = QGraphicsView(scene)
        context.add_widget(view)

        # Publisher timer and start / stop / reset button
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_roi_position)
        self._widget.start_button.clicked.connect(self.start_button_clicked)
        self._widget.reset_button.clicked.connect(self.reset_button_clicked)

        # Publisher
        self.pub = rospy.Publisher('target_region', Polygon , queue_size=10)
        self.rate = rospy.Rate(10)


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
        x = float(self.roi.pos().x())
        y = float(self.roi.pos().y())
        str_pos = str(x) + ", " + str(y)
        self._widget.position_edit.setText(str_pos)
        # print(str_pos)

        z = 0.0
        roi_points = Polygon()
        div = 200
        x1 = x - (float(self.roi_widht) / div)
        y1 = y + (float(self.roi_widht) / div)
        x2 = x + (float(self.roi_widht) / div)
        y2 = y + (float(self.roi_widht) / div)
        x3 = x + (float(self.roi_widht) / div)
        y3 = y - (float(self.roi_widht) / div)
        x4 = x - (float(self.roi_widht) / div)
        y4 = y - (float(self.roi_widht) / div)
        print(x1, y1)
        roi_points.points.append(Point32(x1, y1, z))
        roi_points.points.append(Point32(x2, y2, z))
        roi_points.points.append(Point32(x3, y3, z))
        roi_points.points.append(Point32(x4, y4, z))

        self.pub.publish(roi_points)

    def reset_button_clicked(self):

        self.roi.setPos(0,0)
        rospy.loginfo("Reset vlivked")

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        rospy.loginfo("Exiting..")
        self.pub.unregister()
        self.timer.stop()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
