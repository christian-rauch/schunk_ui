from __future__ import print_function

import os
import rospy
import rospkg

# Qt GUI
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

# action library
import actionlib

# message types
from std_srvs.srv import Trigger
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint

import math


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

        #### connect to ROS
        # action clients
        self.action_client = actionlib.SimpleActionClient('/gripper/sdh_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("waiting for follow_joint_trajectory service...")
        self.action_client.wait_for_server()
        rospy.loginfo("connected to follow_joint_trajectory action")

        #### connect to UI
        # service buttons
        self._widget.button_init.clicked.connect(lambda: self.call_service("init"))
        self._widget.button_recover.clicked.connect(lambda: self.call_service("recover"))
        self._widget.button_stop.clicked.connect(lambda: self.call_service("stop"))
        # joint sliders
        self._widget.proximal_slider.valueChanged.connect(lambda value: self.spinner_update(self._widget.proximal_spinbox, value, True))
        self._widget.distal_slider.valueChanged.connect(lambda value: self.spinner_update(self._widget.distal_spinbox, value, True))
        # joint spinners
        self._widget.proximal_spinbox.valueChanged.connect(lambda value: self.slider_update(self._widget.proximal_slider, value, True))
        self._widget.distal_spinbox.valueChanged.connect(lambda value: self.slider_update(self._widget.distal_slider, value, True))

        # set sliders by default spinner box values
        self.slider_update(self._widget.proximal_slider, self._widget.proximal_spinbox.value(), send_joints=False)
        self.slider_update(self._widget.distal_slider, self._widget.distal_spinbox.value(), send_joints=False)

    def call_service(self, name):
        service_name = '/gripper/sdh_controller/'+name

        rospy.wait_for_service(service_name)

        service = rospy.ServiceProxy(service_name, Trigger)
        resp = service()

        print("Called service:", name)
        print("Response:")
        print(resp)

        return resp.success

    def spinner_update(self, spinner, value, send_joints):
        # just set spinner value, do not forward signal back to slider
        spinner.blockSignals(True)
        spinner.setValue(value/1000.0)
        spinner.blockSignals(False)

        if send_joints:
            self.send_grasp_joint_positions()

    def slider_update(self, slider, value, send_joints):
        # just set slider value, do not forward signal back to spinner
        slider.blockSignals(True)
        slider.setValue(value * 1000)
        slider.blockSignals(False)

        if send_joints:
            self.send_grasp_joint_positions()

    def send_grasp_joint_positions(self):
        # values in range 0 ... 1
        proximal_value = self._widget.proximal_spinbox.value()
        distal_value = self._widget.distal_spinbox.value()

        # define sets of joints
        proximal_joints = ["sdh_thumb_2_joint", "sdh_finger_12_joint", "sdh_finger_22_joint"]
        distal_joints   = ["sdh_thumb_3_joint", "sdh_finger_13_joint", "sdh_finger_23_joint"]
        static_joints   = ["sdh_knuckle_joint"]
        all_joints      = static_joints + proximal_joints + distal_joints

        # map joint ranges from [0..1] to individual set ranges
        # proximal range: [-pi/2 .. 0]
        # distal range: [0 .. pi/2]

        proximal_range  = [-math.pi/2.0, 0.0]
        distal_range    = [0.0, math.pi/2.0]
        proximal_jpos   = proximal_range[0] + proximal_value * (proximal_range[1]-proximal_range[0])
        distal_jpos     = distal_range[0] + distal_value * (distal_range[1] - distal_range[0])

        trajectory_goal = FollowJointTrajectoryGoal()

        # add single single joint point to trajectory
        for jname in all_joints:
            # select joint position from set
            trajpoint = JointTrajectoryPoint()
            if jname in static_joints:
                trajpoint.positions.append(0)
            elif jname in proximal_joints:
                trajpoint.positions.append(proximal_jpos)
            elif jname in distal_joints:
                trajpoint.positions.append(distal_jpos)
            else:
                raise Exception("joint not in set")

            trajectory_goal.trajectory.joint_names.append(jname)
            trajectory_goal.trajectory.points.append(trajpoint)

        # send trajectory and wait for response
        self.action_client.send_goal(trajectory_goal)
        self.action_client.wait_for_result()
        trajectory_result = self.action_client.get_result()

        return trajectory_result.error_code == FollowJointTrajectoryResult.SUCCESSFUL

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.action_client.cancel_all_goals()

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
