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
ROI_STEP = 0.1 # ROI size increment step

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
        self.target_roi = None
        self.interpolated_points = None
        self.new_target = False

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

        # Publisher timer and buttons
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_roi_position)

        self._widget.start_button.clicked.connect(self.start_button_clicked)
        self._widget.reset_button.clicked.connect(self.reset_button_clicked)
        self._widget.decSizeBtn.clicked.connect(self.decrease_roi_size)
        self._widget.incSizeBtn.clicked.connect(self.increase_roi_size)
        self._widget.decHeightBtn.clicked.connect(self.decrease_roi_height)
        self._widget.incHeightBtn.clicked.connect(self.increase_roi_height)
        self._widget.decWidthBtn.clicked.connect(self.decrease_roi_width)
        self._widget.incWidthBtn.clicked.connect(self.increase_roi_width)
        self._widget.speedSlider.valueChanged.connect(self.speed_change)

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

    def speed_change(self):
        value = self._widget.speedSlider.value()

        sliderMin = 1.0
        sliderMax = 99.0
        intervalMin = 150.0
        intervalMax = 10.0
        leftSpan = sliderMax - sliderMin
        rightSpan = intervalMax - intervalMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - sliderMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        interval =  intervalMin + (valueScaled * rightSpan)

        print interval
        self.timer.setInterval(interval)


    def keyPressHandler(self, event):
        if event.key() == Qt.Key_PageUp:
            self.increase_roi_size()
        elif event.key() == Qt.Key_PageDown:
            self.decrease_roi_size()
        elif event.key() == Qt.Key_Up:
            self.increase_roi_height()
        elif event.key() == Qt.Key_Down:
            self.decrease_roi_height()
        elif event.key() == Qt.Key_Left:
            self.decrease_roi_width()
        elif event.key() == Qt.Key_Right:
            self.increase_roi_width()

    def update_roi_position(self):
        if self.target_roi:
            self.update_target_position()
            return

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
        y_up = y - height/2.0
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

    def update_target_position(self):
        if self.new_target:
            self.calculate_points()
            self.new_target = False
            return

        if self.interpolated_points:

            points = self.interpolated_points.pop(0)
            print points
            x = points[0]
            y = points[1]
            width = self.target_roi.rect().width()
            height = self.target_roi.rect().height()

            # Calculate ROI corner points
            roi_points = Polygon()
            x_left = x - width / 2.0
            y_up = y - height / 2.0
            x_right = x + width / 2.0
            y_down = y + height / 2.0

            coords = [(x_left, y_up),
                      (x_right, y_up),
                      (x_right, y_down),
                      (x_left, y_down)]

            z = 0.0
            for coord in coords:
                roi_points.points.append(Point(coord[0], coord[1], z ))

            self.pub.publish(roi_points)
            if len(self.interpolated_points) == 1:
                self.interpolated_points = None


    def calculate_points(self):
        x1 = float(self.roi.pos().x())
        y1 = float(self.roi.pos().y())
        x2 = float(self.target_roi.rect().x() + self.target_roi.rect().width()/2)
        y2 = float(self.target_roi.rect().y() + self.target_roi.rect().height()/2)
        #print x1, y1, x2, y2
        d = 0.01
        d_full = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        s = d / d_full

        self.interpolated_points = []
        a = s
        while a < 1:
            x = (1 - a) * x1 + a * x2
            y = (1 - a) * y1 + a * y2
            self.interpolated_points.append((x, y))
            a += s

        #print "points ", len(self.interpolated_points)
        #print self.interpolated_points

    @pyqtSlot()
    def drawRobots(self):
        for robot in self.robots:
            robot.drawOnScene()

    def reset_button_clicked(self):
        self.roi.setPos(0,0)
        self.roi.setRect(0 - self.roi_width / 2.0, 0 - self.roi_height/2.0 , self.roi_width, self.roi_height)
        if self.target_roi:
            self.scene.removeItem(self.target_roi)
            self.target_roi = None

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
        else:
            if self.target_roi:
                self.scene.removeItem(self.target_roi)
                self.target_roi = None
            else:
                width = self.roi.rect().width()
                height = self.roi.rect().height()
                self.target_roi = self.scene.addRect(event.scenePos().x() - self.roi_width/2.0, event.scenePos().y()
                                                 - self.roi_height/2.0 , width, height,
                                                 QPen(Qt.black, 0.01), QBrush(QColor(0, 255, 0, 50)))
                #self.target_roi.setFlag(QGraphicsItem.ItemIsMovable, True)
                self.new_target = True


    def increase_roi_size(self):
        width = self.roi.rect().width()
        height = self.roi.rect().height()
        self.roi.setRect(-width / 2.0 - ROI_STEP/2, -height / 2.0 - ROI_STEP/2, width + ROI_STEP, height + ROI_STEP)
        if self.target_roi:
            x = self.target_roi.rect().x()
            y = self.target_roi.rect().y()
            self.target_roi.setRect(x - ROI_STEP/2, y - ROI_STEP/2, width + ROI_STEP, height + ROI_STEP)

    def decrease_roi_size(self):
        width = self.roi.rect().width()
        height = self.roi.rect().height()
        self.roi.setRect(-width / 2.0 + ROI_STEP/2, -height / 2.0 + ROI_STEP/2, width - ROI_STEP, height - ROI_STEP)
        if self.target_roi:
            x = self.target_roi.rect().x()
            y = self.target_roi.rect().y()
            self.target_roi.setRect(x + ROI_STEP/2, y + ROI_STEP/2, width - ROI_STEP, height - ROI_STEP)

    def increase_roi_height(self):
        width = self.roi.rect().width()
        height = self.roi.rect().height()
        self.roi.setRect(-width / 2.0, -height / 2.0 - ROI_STEP/2, width, height + ROI_STEP)
        if self.target_roi:
            x = self.target_roi.rect().x()
            y = self.target_roi.rect().y()
            self.target_roi.setRect(x, y - ROI_STEP/2, width, height + ROI_STEP)

    def decrease_roi_height(self):
        width = self.roi.rect().width()
        height = self.roi.rect().height()
        self.roi.setRect(-width / 2.0, -height / 2.0 + ROI_STEP/2, width, height - ROI_STEP)
        if self.target_roi:
            x = self.target_roi.rect().x()
            y = self.target_roi.rect().y()
            self.target_roi.setRect(x, y + ROI_STEP/2, width, height - ROI_STEP)

    def increase_roi_width(self):
        width = self.roi.rect().width()
        height = self.roi.rect().height()
        self.roi.setRect(-width / 2.0 - ROI_STEP/2, -height / 2.0, width + ROI_STEP, height)
        if self.target_roi:
            x = self.target_roi.rect().x()
            y = self.target_roi.rect().y()
            self.target_roi.setRect(x - ROI_STEP/2, y, width + ROI_STEP, height)

    def decrease_roi_width(self):
        width = self.roi.rect().width()
        height = self.roi.rect().height()
        self.roi.setRect(-width / 2.0 + ROI_STEP/2, -height / 2.0, width - ROI_STEP, height)
        if self.target_roi:
            x = self.target_roi.rect().x()
            y = self.target_roi.rect().y()
            self.target_roi.setRect(x + ROI_STEP/2, y, width - ROI_STEP, height)


    def shutdown_plugin(self):
        rospy.loginfo("Exiting")
        self.pub.unregister()
        self.timer.stop()
        # Unsubscribe robots listening odom
        del self.robots
