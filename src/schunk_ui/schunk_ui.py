from __future__ import print_function

import os
import rospy
import rospkg

# Qt GUI
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QImage, QPixmap, QTransform

# action library
import actionlib

# message types
from std_srvs.srv import Trigger
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from schunk_sdh.msg import TemperatureArray, PressureArrayList

import math
import threading
import time
import numpy as np


class SchunkPlugin(Plugin):
    def __init__(self, context):
        super(SchunkPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('SchunkPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('schunk_ui'), 'resource', 'ui', 'SchunkJoints.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('SchunkPluginUI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Connect to ROS
        # action clients
        self.action_client = actionlib.SimpleActionClient('/gripper/sdh_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        # subscribers
        self.sub_temp = rospy.Subscriber("/gripper/sdh_controller/temperature", TemperatureArray, self.on_temp)
        self.sub_tactile = rospy.Subscriber("/gripper/sdh_controller/pressure", PressureArrayList, self.on_tactile)

        # maximum measurable pressure
        # 4096 * calib_pressure / calib_voltage
        # 2^12 * 0.000473 / 592.1 = 0.0032720959297415976 (unit: N/(mm*mm) )
        self.max_pressure = 2**12 * 0.000473 / 592.1

        self.max_pressure_readings = 0

        # Connect to UI
        # service buttons
        self._widget.button_init.clicked.connect(lambda: self.call_service("init"))
        self._widget.button_disconnect.clicked.connect(lambda: self.call_service("shutdown"))
        self._widget.button_estop.clicked.connect(lambda: self.call_service("emergency_stop"))
        self._widget.button_motor_on.clicked.connect(lambda: self.call_service("motor_on"))
        self._widget.button_motor_off.clicked.connect(lambda: self.call_service("motor_off"))

        self._widget.button_reset_max_pressure.clicked.connect(lambda : self.reset_max_pressure_readings())

        # status text
        self.status_message = self._widget.status_message

        # joint sliders
        self._widget.proximal_slider.valueChanged.connect(
            lambda value: self.on_slider_update(self._widget.proximal_spinbox, value))
        self._widget.distal_slider.valueChanged.connect(
            lambda value: self.on_slider_update(self._widget.distal_spinbox, value))
        # joint spinners
        self._widget.proximal_spinbox.valueChanged.connect(
            lambda value: self.on_spinner_update(self._widget.proximal_slider, value))
        self._widget.distal_spinbox.valueChanged.connect(
            lambda value: self.on_spinner_update(self._widget.distal_slider, value))

        # set spinner boxes by default sliders values
        self._widget.proximal_spinbox.setValue(self._widget.proximal_slider.value() / 1000.0)
        self._widget.distal_spinbox.setValue(self._widget.distal_slider.value() / 1000.0)

        # map temperature names to spinner boxes
        self.tempspinners = dict()
        self.tempspinners["root"]       = self._widget.spin_root
        self.tempspinners["controller"] = self._widget.spin_ctrl
        self.tempspinners["pcb"]        = self._widget.spin_pcb

        self.is_initialised = False
        self.is_motor_on = False
        self.has_new_data = False

        # start working thread
        self.running = True
        self.thread = threading.Thread(target=self.loop, args=())
        self.thread.start()

    def on_slider_update(self, spinner, value):
        # just set spinner value, do not forward signal back to slider
        spinner.blockSignals(True)
        spinner.setValue(value / 1000.0)
        spinner.blockSignals(False)
        self.has_new_data = True

    def on_spinner_update(self, slider, value):
        # just set slider value, do not forward signal back to spinner
        slider.blockSignals(True)
        slider.setValue(value * 1000)
        slider.blockSignals(False)
        self.has_new_data = True

    def call_service(self, name):
        service_name = '/gripper/sdh_controller/' + name

        try:
            rospy.wait_for_service(service_name, timeout=0.5)
        except rospy.exceptions.ROSException:
            rospy.logerr("service '" + str(name) + "' is not available")
            self.status_message.setText("service '" + str(name) + "' is not available")
            return False

        service = rospy.ServiceProxy(service_name, Trigger)
        resp = service()

        print("Called service:", name)
        print("Response:")
        print(resp)
        self.status_message.setText(resp.message)

        if name == "init":
            self.is_initialised = resp.success
            self.is_motor_on = resp.success
        elif name == "shutdown":
            self.is_initialised = not resp.success
            self.is_motor_on = not resp.success

        if name == "motor_on":
            self.is_motor_on = resp.success
        elif name == "motor_off":
            self.is_motor_on = not resp.success

        if resp.success and (name in ["init", "motor_on"]):
            self.has_new_data = True

        return resp.success

    def loop(self):
        self.running = True
        while self.running:
            if self.is_initialised and self.is_motor_on and self.has_new_data:
                self.send_grasp_joint_positions()
                self.has_new_data = False
            time.sleep(0.1)

    def send_grasp_joint_positions(self):
        # values in range 0 ... 1
        proximal_value = self._widget.proximal_spinbox.value()
        distal_value = self._widget.distal_spinbox.value()

        # define sets of joints
        proximal_joints = ["sdh_thumb_2_joint", "sdh_finger_12_joint", "sdh_finger_22_joint"]
        distal_joints = ["sdh_thumb_3_joint", "sdh_finger_13_joint", "sdh_finger_23_joint"]
        static_joints = ["sdh_knuckle_joint"]
        all_joints = static_joints + proximal_joints + distal_joints

        # map joint ranges from [0..1] to individual set ranges
        # proximal range: [-pi/2 .. 0]
        # distal range: [0 .. pi/2]

        proximal_range = [-math.pi / 2.0, 0.0]
        distal_range = [0.0, math.pi / 2.0]
        proximal_jpos = proximal_range[0] + proximal_value * (proximal_range[1] - proximal_range[0])
        distal_jpos = distal_range[0] + distal_value * (distal_range[1] - distal_range[0])

        trajectory_goal = FollowJointTrajectoryGoal()

        # add single single joint point to trajectory
        trajectory_goal.trajectory.points.append(JointTrajectoryPoint())
        for jname in all_joints:
            trajectory_goal.trajectory.joint_names.append(jname)
            # select joint position from set
            if jname in static_joints:
                trajectory_goal.trajectory.points[0].positions.append(0)
            elif jname in proximal_joints:
                trajectory_goal.trajectory.points[0].positions.append(proximal_jpos)
            elif jname in distal_joints:
                trajectory_goal.trajectory.points[0].positions.append(distal_jpos)
            else:
                raise Exception("joint not in set")

        # send trajectory and wait for response
        self.action_client.send_goal(trajectory_goal)

        if self.action_client.wait_for_result(timeout=rospy.Duration(5.0)):
            trajectory_result = self.action_client.get_result()
            print("set joints to "+('%s' % trajectory_goal.trajectory.points[0].positions))
            self.status_message.setText("set joints")
            return trajectory_result.error_code == FollowJointTrajectoryResult.SUCCESSFUL
        else:
            rospy.logerr("timeout while waiting for response from grasp goal")
            self.status_message.setText("timeout while waiting for response from grasp goal")
            return False

    def on_temp(self, temp_msg):
        if self.is_initialised:
            temps = dict(zip(temp_msg.name, temp_msg.temperature))
            for name, spinner in self.tempspinners.iteritems():
                try:
                    spinner.setValue(temps[name])
                except KeyError:
                    rospy.logerr("temperature",name,"is not provided by SDH driver node")

    @staticmethod
    def jet(m):
        # clip values to range [0,1]
        m = np.clip(m, 0.0, 1.0)
        r = np.clip(np.minimum(4*m-1.5, -4*m+4.5), 0.0, 1.0)
        g = np.clip(np.minimum(4*m-0.5, -4*m+3.5), 0.0, 1.0)
        b = np.clip(np.minimum(4*m+0.5, -4*m+2.5), 0.0, 1.0)
        return (np.dstack((r,g,b))*255).astype(np.uint8)

    def on_tactile(self, msg_tactile):
        for m in msg_tactile.pressure_list:
            p = np.array(m.pressure, dtype=np.float64).reshape((m.cells_y, m.cells_x))
            # apply colour map to scaled values
            im = SchunkPlugin.jet(p / self.max_pressure)

            getattr(self._widget, "lbl_"+m.sensor_name).setText(m.sensor_name+":")
            getattr(self._widget, "max_"+m.sensor_name).setText("{:.9f}".format(np.max(p)))

            if np.max(p) > self.max_pressure_readings:
                self.max_pressure_readings = np.max(p)
                self._widget.max_pressure.setText(str(self.max_pressure_readings))

            # label for tactile content
            lbl = getattr(self._widget, "tactile_"+m.sensor_name)
            qimg = QImage(im.data, im.shape[1], im.shape[0], 3*im.shape[1], QImage.Format_RGB888)
            T = QTransform()
            T.rotate(90)
            qpix = QPixmap.fromImage(qimg).transformed(T)
            qpix = qpix.scaledToWidth(qpix.width()*20)
            lbl.setPixmap(qpix)

    def reset_max_pressure_readings(self):
        self.max_pressure_readings = 0
        self._widget.max_pressure.setText(str(self.max_pressure_readings))

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.sub_temp.unregister()
        self.sub_tactile.unregister()

        self.action_client.cancel_all_goals()
        self.running = False
        self.thread.join()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

        # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
