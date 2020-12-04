import os
import rospy
import rospkg

from geometry_msgs.msg import Polygon
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
        self.roi = scene.addRect(0,0,200,200, QPen(Qt.black), QBrush(Qt.gray))
        self.roi.setFlag(QGraphicsItem.ItemIsMovable, True)
        view = QGraphicsView(scene)
        context.add_widget(view)

        # Publisher timer and start / stop button
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_roi_position)
        self._widget.start_button.clicked.connect(self.start_button_clicked)

        # Publisher
        self.pub = rospy.Publisher('ROI_points', String , queue_size=10)
        self.rate = rospy.Rate(10)


    def start_button_clicked(self):
        if self._widget.start_button.text() == "Stop":
            self.timer.stop()
            self._widget.start_button.setText("Start")
        else:
            self._widget.start_button.setText("Stop")
            interval = self._widget.interval_spinbox.value()
            self.timer.start(interval)

    def update_roi_position(self):


        pos = ( self.roi.pos().x(), self.roi.pos().y() )
        self._widget.position_edit.setText(str(pos[0]) + ", " + str(pos[1]))
        print(pos)

        self.pub.publish("asd")
        self.rate.sleep()

    def shutdown_plugin(self):
        # TODO unregister all publishers here
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
