#!/usr/bin/env python
import copy
import sys
from std_msgs.msg import String
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
import tf


class MoveGroupPythonInteface(object):
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_manipulator', anonymous=True)

        # Globar variables
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('manipulator_i5')
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.planning_frame = self.group.get_planning_frame()
        self.group_names = self.robot.get_group_names()

        # Robot configuration
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.1)
        self.group.set_planning_time(0.1)
        self.group.set_max_acceleration_scaling_factor(.5)
        self.group.set_max_velocity_scaling_factor(.5)

    def do_random(self, coordinates):
        waypoints = []

        wpose = geometry_msgs.msg.Pose()
        q = tf.transformations.quaternion_from_euler(3.12, 0.015, 0.269)

        wpose.position.x = coordinates[0]
        wpose.position.y = coordinates[1]
        wpose.position.z = coordinates[2]

        wpose.orientation.x = q[0]
        wpose.orientation.y = q[1]
        wpose.orientation.z = q[2]
        wpose.orientation.w = q[3]
        waypoints.append(copy.deepcopy(wpose))

        self.plan_and_execute(waypoints)

    def plan_and_execute(self, goal):
        self.group.set_start_state_to_current_state()
        plan, fraction = self.group.compute_cartesian_path(goal, 0.01, 0.0, True)

        self.group.execute(plan, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()


    def test_run(self):
        print "**********STARTING TEST RUN**********"
        self.do_random((-0.0824, -0.69778, 0.0478))
        # self.move_to_coordinates()
        print "**********TEST RUN END**********"


def main():
    try:
        mgpi = MoveGroupPythonInteface()
        mgpi.test_run()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()