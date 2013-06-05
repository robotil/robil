import roslib; roslib.load_manifest('C42_DynamicLocomotion')
from std_msgs.msg import Float64,Bool
from sensor_msgs.msg import *
import rospy, math, sys,numpy
from IKException import IKReachException
from geometry_msgs.msg import *
import copy
from atlas_msgs.msg import ForceTorqueSensors
from foot_contact_filter import contact_filter
from contact_reflex import contact_reflex
import tf
import pylab as pl