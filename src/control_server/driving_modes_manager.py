# ROS launch files
# http://wiki.ros.org/roslaunch/API%20Usage
import roslaunch

from network_status import get_command_center_ip

class DrivingModesManager():
  def __init__(self):
    self.active_launch_file = None
    self.active_mode_name = ""
  def terminate_active_mode(self):
      if(self.active_launch_file is not None):
        self.active_launch_file.shutdown()
        print "terminating {} mode".format(self.active_mode_name) 
  
  def start_manual_mode(self):
    self.terminate_active_mode()
    self.active_mode_name="maunal mode"
    print "starting {}".format(self.active_mode_name)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    self.active_launch_file = roslaunch.parent.ROSLaunchParent(uuid, ["../catkin-ws/src/kotyambaCar/launch/manual_mode.launch"])
    self.active_launch_file.start()
    
  def start_training_mode(self):
    self.terminate_active_mode()
    self.active_mode_name="training mode"
    print "starting {}".format(self.active_mode_name)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    try:
      get_command_center_ip()
    except Exception as e:
      print e
      print "Can not start training mode! Running command center machine is required"
    else:
      self.active_launch_file = roslaunch.parent.ROSLaunchParent(uuid, ["../catkin-ws/src/kotyambaCar/launch/training_mode.launch"])
      self.active_launch_file.start()
      rospy.loginfo("started training mode")
  
  def start_self_driving_mode(self):
    self.terminate_active_mode()
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    self.active_launch_file = roslaunch.parent.ROSLaunchParent(uuid, ["../catkin-ws/src/kotyambaCar/launch/self_driving_mode.launch"])
    self.active_mode_name="self-driving mode"
    print "starting {}".format(self.active_mode_name)
    self.active_launch_file.start()
    rospy.loginfo("{} mode launch file started").format(self.active_mode_name)