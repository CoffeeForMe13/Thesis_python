import numpy as np
import array
import csv
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

'''
def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True
#'''


class PythonCodeForThesis(object):
    """Thesis"""
    #"""
    def __init__(self):
        super(PythonCodeForThesis, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('rcs_project', anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

    
        planning_frame = move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Available Planning Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
    #"""



    def go_to_A(self,qVect):
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = qVect[0]
        joint_goal[1] = qVect[1]
        joint_goal[2] = qVect[2]
        joint_goal[3] = qVect[3]
        joint_goal[4] = qVect[4]
        joint_goal[5] = qVect[5]

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)



    
def move_ur5_joint_trajectory(xopt, duration=0.1):

    #global jt_pub_ur5
    jt_ur5 = trajectory_msgs.msg.JointTrajectory()
    jt_ur5.joint_names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
                    'wrist_1_joint','wrist_2_joint','wrist_3_joint']

    jtpt = trajectory_msgs.msg.JointTrajectoryPoint()
    jtpt.positions = [xopt[0], xopt[1], xopt[2], xopt[3], xopt[4], xopt[5]]
    jtpt.time_from_start = rospy.Duration.from_sec(duration)

    jt_ur5.points.append(jtpt)
    return jt_ur5
    #jt_pub_ur5.publish(jt_ur5) 
#"""


def main():
    try:
        print("============ Press `Enter` to begin ...")
        raw_input()
        ur5Method = PythonCodeForThesis()

        # Read the csv file
        reader = csv.reader(open("qc.csv", "rb"), delimiter=",")
        x = list(reader)
        jointValues = np.array(x).astype("float")
        print(jointValues)
        #

        #ur5Method.go_to_A(jointValues[0,:])

        """
        goodOlAux = JointTrajectory()
        print(goodOlAux)
        #"""


        print("============ Simulation complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()