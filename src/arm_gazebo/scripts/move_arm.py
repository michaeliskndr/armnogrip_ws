#!/usr/bin/env python

import csv
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

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

class MoveGroupPythonRobotArm(object):
  def __init__(self):
    super(MoveGroupPythonRobotArm, self).__init__()
    
    #Initialisasi moveit commander and rospy node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_robot_arm',
                    anonymous=True)

    #Initialisasi Robot Object
    robot = moveit_commander.RobotCommander()

    #Initialisasi Planning Interface
    scene = moveit_commander.PlanningSceneInterface()

    #Initialisasi Move Group Commander
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    #Initialisasi Display Trajectory
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory, 
                                                   queue_size=20)

    #Variables Declaration
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher

    self.index = 0
    self.pos_index = []

    self.pos_x = []
    self.pos_y = []
    self.pos_z = []
    self.joint_values = []

    #Joint Var

    self.joint_1 = []
    self.joint_2 = []
    self.joint_3 = []
    self.joint_4 = []
    self.joint_5 = []
    self.joint_6 = []

    #RPY Var
    self.roll_list = []
    self.pitch_list = []
    self.yaw_list = []

  def go_to_standby(self):
    group = self.group
    
    #joint current values
    joint_values = group.get_current_joint_values()

    #Setting Joint Goals
    joint_goal = [0, 0, 0, 0, 0, 0]

    group.go(joint_goal, wait=True)

    #Clear Residual Movement
    group.stop()
    group.clear_pose_targets()

    #Current joints
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01) 

  def go_to_joint(self):
    group = self.group
    
    #joint current values
    joint_values = group.get_current_joint_values()

    #Setting Joint Goals
    joint_goal = [0, 0, pi/4, 0, pi/4, 0] #Trial Number 1
    #joint_goal = [pi/2, pi/3, pi/3, pi/2, pi/3, 0.1] #Trial Number 3
    #joint_goal = [pi/2, 0, pi/3, 0, pi/3, 0.05] #Trial Number 4
    #joint_goal = [pi/3, pi/3, pi/4, pi/3, pi/3, 0.05] #Trial number 5
    #joint_goal = [0, pi/3, pi/4, 0, pi/4, 0.05] #Trial Number 2
    #joint_goal = [0, 0, 0, 0, 0, 0]
    #joint_goal = [-0.5936727538985871,-0.37959710213201037,-0.5393352081716776,3.042571110190201,0.21357126375962907,0.09447648271816508]
    joint_goal = [-2.994477850296225,-0.9121160316819198,0.5429637163676598,2.339947109094276,1.4947898161008863,0.00034897988504223424]
    #Execute Movement
    group.go(joint_goal, wait=True)

    #Clear Residual Movement
    group.stop()
    group.clear_pose_targets()

    #Input Data
    self.insert_data_to_list()
    #Current joints
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_position(self):
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()

    position_goal = [-0.16,-0.3,0.72] #Position 1 pos3.csv
    # position_goal = [-0.205, 0.527, 0.441] #Position 2 pos2.csv
    # position_goal = [0.652, 0.178, 0.584] #Position 3 pos4.csv
    # position_goal = [-0.028, -0.182, 0.589] #Position 4
    # position_goal = [0.467, -0.262, 0.688] #pos5.csv Position 5
    # position_goal = [0.074, -0.406, 0.707] #pos 6.csv
    # position_goal = [-0.561, -0.302, 0.321] #pos7.csv
    # position_goal = [0.121, 0.212, 0.813] #pos8.csv
    # position_goal = [0.356, 0.471, 0.213] #pos9.csv
    # position_goal = [0.262, 0.613, 0.352] #pos10.csv
    #position_goal = [0.483, 0.467, 0.461] #pos11.cv

    position_execute = group.set_position_target(position_goal)
    group.go(position_execute, wait=True)

    group.stop()

    group.clear_pose_targets()

    #Position Data Inputting and returning current pose
    current_pose = self.insert_data_to_list()
    # print current_pose

    return all_close(pose_goal, current_pose, 0.01)

  def go_to_position2(self):
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()

    # position_goal = [-0.16,-0.3,0.72] #Position 1 pos3.csv
    position_goal = [-0.205, 0.527, 0.441] #Position 2 pos2.csv
    # position_goal = [0.652, 0.178, 0.584] #Position 3 pos4.csv
    # position_goal = [-0.028, -0.182, 0.589] #Position 4
    # position_goal = [0.467, -0.262, 0.688] #pos5.csv Position 5
    # position_goal = [0.074, -0.406, 0.707] #pos 6.csv
    # position_goal = [-0.561, -0.302, 0.321] #pos7.csv
    # position_goal = [0.121, 0.212, 0.813] #pos8.csv
    # position_goal = [0.356, 0.471, 0.213] #pos9.csv
    # position_goal = [0.262, 0.613, 0.352] #pos10.csv
    # position_goal = [0.483, 0.467, 0.461]

    position_execute = group.set_position_target(position_goal)
    group.go(position_execute, wait=True)

    group.stop()

    group.clear_pose_targets()

    #Position Data Inputting and returning current pose
    current_pose = self.insert_data_to_list()
    # print current_pose

    return all_close(pose_goal, current_pose, 0.01)

  def go_to_position3(self):
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()

    # position_goal = [-0.16,-0.3,0.72] #Position 1 pos3.csv
    # position_goal = [-0.205, 0.527, 0.441] #Position 2 pos2.csv
    position_goal = [0.652, 0.178, 0.584] #Position 3 pos4.csv
    # position_goal = [-0.028, -0.182, 0.589] #Position 4
    # position_goal = [0.467, -0.262, 0.688] #pos5.csv Position 5
    # position_goal = [0.074, -0.406, 0.707] #pos 6.csv
    # position_goal = [-0.561, -0.302, 0.321] #pos7.csv
    # position_goal = [0.121, 0.212, 0.813] #pos8.csv
    # position_goal = [0.356, 0.471, 0.213] #pos9.csv
    # position_goal = [0.262, 0.613, 0.352] #pos10.csv
    # position_goal = [0.483, 0.467, 0.461]

    position_execute = group.set_position_target(position_goal)
    group.go(position_execute, wait=True)

    group.stop()

    group.clear_pose_targets()

    #Position Data Inputting and returning current pose
    current_pose = self.insert_data_to_list()
    # print current_pose

    return all_close(pose_goal, current_pose, 0.01)

  def go_to_position4(self):
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()

    # position_goal = [-0.16,-0.3,0.72] #Position 1 pos3.csv
    # position_goal = [-0.205, 0.527, 0.441] #Position 2 pos2.csv
    # position_goal = [0.652, 0.178, 0.584] #Position 3 pos4.csv
    position_goal = [-0.028, -0.182, 0.589] #Position 4
    # position_goal = [0.467, -0.262, 0.688] #pos5.csv Position 5
    # position_goal = [0.074, -0.406, 0.707] #pos 6.csv
    # position_goal = [-0.561, -0.302, 0.321] #pos7.csv
    # position_goal = [0.121, 0.212, 0.813] #pos8.csv
    # position_goal = [0.356, 0.471, 0.213] #pos9.csv
    # position_goal = [0.262, 0.613, 0.352] #pos10.csv
    # position_goal = [0.483, 0.467, 0.461]

    position_execute = group.set_position_target(position_goal)
    group.go(position_execute, wait=True)

    group.stop()

    group.clear_pose_targets()

    #Position Data Inputting and returning current pose
    current_pose = self.insert_data_to_list()
    # print current_pose

    return all_close(pose_goal, current_pose, 0.01)

  def go_to_position5(self):
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()

    # position_goal = [-0.16,-0.3,0.72] #Position 1 pos3.csv
    # position_goal = [-0.205, 0.527, 0.441] #Position 2 pos2.csv
    # position_goal = [0.652, 0.178, 0.584] #Position 3 pos4.csv
    # position_goal = [-0.028, -0.182, 0.589] #Position 4
    position_goal = [0.467, -0.262, 0.688] #pos5.csv Position 5
    # position_goal = [0.074, -0.406, 0.707] #pos 6.csv
    # position_goal = [-0.561, -0.302, 0.321] #pos7.csv
    # position_goal = [0.121, 0.212, 0.813] #pos8.csv
    # position_goal = [0.356, 0.471, 0.213] #pos9.csv
    # position_goal = [0.262, 0.613, 0.352] #pos10.csv
    # position_goal = [0.483, 0.467, 0.461]

    position_execute = group.set_position_target(position_goal)
    group.go(position_execute, wait=True)

    group.stop()

    group.clear_pose_targets()

    #Position Data Inputting and returning current pose
    current_pose = self.insert_data_to_list()
    # print current_pose

    return all_close(pose_goal, current_pose, 0.01)

  def go_to_position6(self):
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()

    # position_goal = [-0.16,-0.3,0.72] #Position 1 pos3.csv
    # position_goal = [-0.205, 0.527, 0.441] #Position 2 pos2.csv
    # position_goal = [0.652, 0.178, 0.584] #Position 3 pos4.csv
    # position_goal = [-0.028, -0.182, 0.589] #Position 4
    # position_goal = [0.467, -0.262, 0.688] #pos5.csv Position 5
    position_goal = [0.074, -0.406, 0.707] #pos 6.csv
    # position_goal = [-0.561, -0.302, 0.321] #pos7.csv
    # position_goal = [0.121, 0.212, 0.813] #pos8.csv
    # position_goal = [0.356, 0.471, 0.213] #pos9.csv
    # position_goal = [0.262, 0.613, 0.352] #pos10.csv
    # position_goal = [0.483, 0.467, 0.461]

    position_execute = group.set_position_target(position_goal)
    group.go(position_execute, wait=True)

    group.stop()

    group.clear_pose_targets()

    #Position Data Inputting and returning current pose
    current_pose = self.insert_data_to_list()
    # print current_pose

    return all_close(pose_goal, current_pose, 0.01)

  def go_to_position7(self):
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()

    # position_goal = [-0.16,-0.3,0.72] #Position 1 pos3.csv
    # position_goal = [-0.205, 0.527, 0.441] #Position 2 pos2.csv
    # position_goal = [0.652, 0.178, 0.584] #Position 3 pos4.csv
    # position_goal = [-0.028, -0.182, 0.589] #Position 4
    # position_goal = [0.467, -0.262, 0.688] #pos5.csv Position 5
    # position_goal = [0.074, -0.406, 0.707] #pos 6.csv
    position_goal = [-0.561, -0.302, 0.321] #pos7.csv
    # position_goal = [0.121, 0.212, 0.813] #pos8.csv
    # position_goal = [0.356, 0.471, 0.213] #pos9.csv
    # position_goal = [0.262, 0.613, 0.352] #pos10.csv
    # position_goal = [0.483, 0.467, 0.461]

    position_execute = group.set_position_target(position_goal)
    group.go(position_execute, wait=True)

    group.stop()

    group.clear_pose_targets()

    #Position Data Inputting and returning current pose
    current_pose = self.insert_data_to_list()
    # print current_pose

    return all_close(pose_goal, current_pose, 0.01)

  def go_to_position8(self):
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()

    # position_goal = [-0.16,-0.3,0.72] #Position 1 pos3.csv
    # position_goal = [-0.205, 0.527, 0.441] #Position 2 pos2.csv
    # position_goal = [0.652, 0.178, 0.584] #Position 3 pos4.csv
    # position_goal = [-0.028, -0.182, 0.589] #Position 4
    # position_goal = [0.467, -0.262, 0.688] #pos5.csv Position 5
    # position_goal = [0.074, -0.406, 0.707] #pos 6.csv
    # position_goal = [-0.561, -0.302, 0.321] #pos7.csv
    position_goal = [0.121, 0.212, 0.813] #pos8.csv
    # position_goal = [0.356, 0.471, 0.213] #pos9.csv
    # position_goal = [0.262, 0.613, 0.352] #pos10.csv
    # position_goal = [0.483, 0.467, 0.461]

    position_execute = group.set_position_target(position_goal)
    group.go(position_execute, wait=True)

    group.stop()

    group.clear_pose_targets()

    #Position Data Inputting and returning current pose
    current_pose = self.insert_data_to_list()
    # print current_pose

    return all_close(pose_goal, current_pose, 0.01)

  def go_to_position9(self):
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()

    # position_goal = [-0.16,-0.3,0.72] #Position 1 pos3.csv
    # position_goal = [-0.205, 0.527, 0.441] #Position 2 pos2.csv
    # position_goal = [0.652, 0.178, 0.584] #Position 3 pos4.csv
    # position_goal = [-0.028, -0.182, 0.589] #Position 4
    # position_goal = [0.467, -0.262, 0.688] #pos5.csv Position 5
    # position_goal = [0.074, -0.406, 0.707] #pos 6.csv
    # position_goal = [-0.561, -0.302, 0.321] #pos7.csv
    # position_goal = [0.121, 0.212, 0.813] #pos8.csv
    position_goal = [0.356, 0.471, 0.213] #pos9.csv
    # position_goal = [0.262, 0.613, 0.352] #pos10.csv
    # position_goal = [0.483, 0.467, 0.461]

    position_execute = group.set_position_target(position_goal)
    group.go(position_execute, wait=True)

    group.stop()

    group.clear_pose_targets()

    #Position Data Inputting and returning current pose
    current_pose = self.insert_data_to_list()
    # print current_pose

    return all_close(pose_goal, current_pose, 0.01)

  def go_to_position10(self):
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()

    # position_goal = [-0.16,-0.3,0.72] #Position 1 pos3.csv
    # position_goal = [-0.205, 0.527, 0.441] #Position 2 pos2.csv
    # position_goal = [0.652, 0.178, 0.584] #Position 3 pos4.csv
    # position_goal = [-0.028, -0.182, 0.589] #Position 4
    # position_goal = [0.467, -0.262, 0.688] #pos5.csv Position 5
    # position_goal = [0.074, -0.406, 0.707] #pos 6.csv
    # position_goal = [-0.561, -0.302, 0.321] #pos7.csv
    # position_goal = [0.121, 0.212, 0.813] #pos8.csv
    # position_goal = [0.356, 0.471, 0.213] #pos9.csv
    # position_goal = [0.262, 0.613, 0.352] #pos10.csv
    position_goal = [0.483, 0.467, 0.461]

    position_execute = group.set_position_target(position_goal)
    group.go(position_execute, wait=True)

    group.stop()

    group.clear_pose_targets()

    #Position Data Inputting and returning current pose
    current_pose = self.insert_data_to_list()
    # print current_pose

    return all_close(pose_goal, current_pose, 0.01)

  def insert_data_to_list(self):
    group = self.group

    #Position Data Inputting
    current_pose = group.get_current_pose().pose
    x = group.get_current_pose().pose.position.x
    y = group.get_current_pose().pose.position.y
    z = group.get_current_pose().pose.position.z
   
    self.index += 1
    self.pos_index.append(self.index)
    self.pos_x.append(x)
    self.pos_y.append(y)
    self.pos_z.append(z)

    joint_values = group.get_current_joint_values()
    joint_1 = joint_values[0]
    joint_2 = joint_values[1]
    joint_3 = joint_values[2]
    joint_4 = joint_values[3]
    joint_5 = joint_values[4]
    joint_6 = joint_values[5]

    self.joint_1.append(joint_1)
    self.joint_2.append(joint_2)
    self.joint_3.append(joint_3)
    self.joint_4.append(joint_4)
    self.joint_5.append(joint_5)
    self.joint_6.append(joint_6)
    
    rpy = group.get_current_rpy()
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]

    self.roll_list.append(roll)
    self.pitch_list.append(pitch)
    self.yaw_list.append(yaw)

    return current_pose

  def get_pos_value(self):
    print self.pos_x, self.pos_y, self.pos_z
  
  def get_current_joint_values(self):
    print self.joint_values
    print self.joint_1, self.joint_2, self.joint_3, self.joint_4, self.joint_5, self.joint_6

  def writejoint_to_csv(self):
    joint_values = self.joint_values
    pos_index = self.pos_index
    rows = list(zip(pos_index, joint_values))

    with open('joint.csv', 'wb') as f:
      writer = csv.writer(f)
      for row in rows:
        writer.writerow(row)

  def writepos_to_csv(self):
    pos_x = self.pos_x
    pos_y = self.pos_y
    pos_z = self.pos_z
    pos_index = self.pos_index
    joint_1 = self.joint_1
    joint_2 = self.joint_2
    joint_3 = self.joint_3
    joint_4 = self.joint_4
    joint_5 = self.joint_5
    joint_6 = self.joint_6
    roll = self.roll_list
    pitch = self.pitch_list
    yaw = self.yaw_list
    rows = list(zip(pos_index, pos_x, pos_y, pos_z, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, roll, pitch, yaw))
   
    with open('positionTrial.csv', 'wb') as f:
      writer = csv.writer(f)
      writer.writerow(['id', 'pos_x', 'pos_y', 'pos_z', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'roll', 'pitch', 'yaw'])
      for row in rows:
        writer.writerow(row)

def main():
  try:
    print "============ Set up Moveit Commander by Pressing 'Enter' ==================="
    raw_input()
    move = MoveGroupPythonRobotArm()

    #print "============ Press `Enter` to execute a movement using a joint goal ..."
    #raw_input()
    #move.go_to_joint()


    #print "============ Press `Enter` to execute a movement using a pose goal ..."
    #raw_input()


    #move.go_to_position()
    print "=================== Execute Movement by Pressing 'Enter' ==================="
    raw_input()
    for i in range(1):
      move.go_to_standby()
      #move.go_to_joint()
      move.go_to_position()
      move.go_to_position2()
      move.go_to_position3()
      # move.go_to_position4()
      # move.go_to_position5()
      # move.go_to_position6()
      # move.go_to_position7()
      # move.go_to_position8()
      # move.go_to_position9()
      # move.go_to_position10()

      #move.get_pos_value()
    print "=================== Finish Executing ==================="
    
    # move.writepos_to_csv()
    # print "csv successful"

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

