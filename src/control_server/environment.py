import subprocess
import os
def setup_distributed_ROS_environment():
  ''' setup environemnt vars required for ROS nodes to work on different machines,
      one machine is setup as master, on all other ROS_IP should be set and ROS_Master
  '''
  print "#### setting env vars for ROS distributed system to work (Running nodes on different machines) ####"
  raspberry_pi_ip = subprocess.check_output(["getent", "ahosts", "raspberrypi.local"]).split()[0]
  os.environ["ROS_IP"] = raspberry_pi_ip
  print "ROS_IP:{}".format(os.environ["ROS_IP"])
  os.environ["ROS_MASTER_IP"] = raspberry_pi_ip
  print "ROS_MASTER_IP:{}".format(os.environ["ROS_MASTER_IP"])
  os.environ["ROS_MASTER_URI"] = "http://{}:11311".format(raspberry_pi_ip)
  print "ROS_MASTER_URI:{}".format(os.environ["ROS_MASTER_URI"])

  KOTYAMBA_REPO_RASPBERRY = subprocess.check_output(["git", "rev-parse", "--show-toplevel"])
  os.environ["KOTYAMBA_REPO_RASPBERRY"] = KOTYAMBA_REPO_RASPBERRY
  print "KOTYAMBA_REPO_RASPBERRY: {}".format(os.environ["KOTYAMBA_REPO_RASPBERRY"])