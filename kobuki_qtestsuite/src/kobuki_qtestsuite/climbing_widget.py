import os
from QtCore import Signal, Slot
from QtGui import QWidget

import roslib
roslib.load_manifest('kobuki_qtestsuite')
import rospy

from python_qt_binding import loadUi
#from python_qt_binding.QtGui import QWidget
from rqt_py_common.extended_combo_box import ExtendedComboBox

from geometry_msgs.msg import Twist

# Local resource imports
import resources.common
import resources.climbing

class ClimbingWidget(QWidget):

    def __init__(self, parent=None):
        super(ClimbingWidget, self).__init__(parent)
        
        # get path to UI file which is a sibling of this file
        # in this example the .ui file is in the same folder as this Python file
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ui', 'climbing.ui')
        # extend the widget with all attributes and children from UI file
        loadUi(ui_file, self, {'ExtendedComboBox': ExtendedComboBox})

        # give QObjects reasonable names
        self.setObjectName('ClimbingUi')
        
        _, _, topic_types = rospy.get_master().getTopicTypes()
        topics = [ topic[0] for topic in topic_types if topic[1] == 'geometry_msgs/Twist' ]
        self.combo_box_topic.setItems.emit(sorted(topics))
        self.cmd_vel_publisher = rospy.Publisher(str(self.combo_box_topic.currentText()), Twist)
        
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def shutdown(self):
        self.cmd_vel_publisher.unregister()
    
    ##########################################################################
    # Slot Callbacks
    ##########################################################################
    @Slot(str)
    def on_topic_combo_box_currentIndexChanged(self, topic_name):
        # This is probably a bit broken, need to test with more than just /cmd_vel so
        # there is more than one option.
        self.cmd_vel_publisher = rospy.Publisher(str(self.combo_box_topic.currentText()), Twist)

    @Slot()
    def on_start_button_clicked(self):
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        print "Start"

    @Slot()
    def on_stop_button_clicked(self):
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        print "stop"
