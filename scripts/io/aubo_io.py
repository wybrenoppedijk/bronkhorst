import rospy
import bronkhorst.scripts.io.

FUN_SET_DIGITAL_OUT = 1
FUN_SET_ANALOG_OUT = 3


class IoStaus:

    def __init__(self):
        rospy.wait_for_service('/aubo_driver/set_io')
        try:
            io_service = rospy.ServiceProxy('add_two_ints', SetIO)