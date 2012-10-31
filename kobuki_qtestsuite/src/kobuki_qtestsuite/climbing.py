import os
import roslib
roslib.load_manifest('kobuki_qtestsuite')
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

class Climbing(Plugin):

    def __init__(self, context):
        super(Climbing, self).__init__(context)
        # give QObjects reasonable names
        self.setObjectName('Climbing')

        # create QWidget
        self._widget = QWidget()
        # get path to UI file which is a sibling of this file
        # in this example the .ui file is in the same folder as this Python file
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ui', 'climbing.ui')
        print ui_file
        # extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # give QObjects reasonable names
        self._widget.setObjectName('ClimbingUi')
        # add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
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
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a dialog to offer the user a set of configuration