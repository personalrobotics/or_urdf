#!/usr/bin/env python
import time
import math
import numpy
import roslib; roslib.load_manifest("orcdchomp")
import ros_exportenv; ros_exportenv.ros_exportenv()
import openravepy
import libprrave.ik
import libprrave.kin
import libprrave.rave
import libprrave.tsr

# Load the environment
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
e = openravepy.Environment()
e.SetViewer('qtcoin')
e.Load('robots/barrettwam.robot.xml')

# set up robot, active manip, active dofs
r = e.GetRobot('BarrettWAM')
r.SetActiveManipulator('arm')
m = r.GetActiveManipulator()
r.SetActiveDOFs(m.GetArmIndices())

# Create the module
libprrave.rave.load_module(e, 'orcdchomp', 'blah_load_string')

r.SetActiveDOFValues([-0.5,1.0,0.0,2.0,0.0,-1.0,0.0])

#e.Load('objects/misc/coordframe.kinbody.xml')
#frame = e.GetKinBody('coordframe')
#frame.SetTransform(r.GetLink('wam0').GetTransform())

e.Load('objects/misc/liftingbox.kinbody.xml')
box = e.GetKinBody('box')
Tbox = box.GetTransform()
Tbox[0:3,3] = [0.6, 0.0, -0.25]
box.SetTransform(Tbox)

# Disable the robot from collision checking while computing distance field 
raw_input('Press [Enter] compute distance field ...')
r.Enable(False)
libprrave.rave.get_module(e,'orcdchomp').SendCommand(
   'computedistancefield robot BarrettWAM')
r.Enable(True)

raw_input('Press [Enter] run chomp ...')
t_data = libprrave.rave.get_module(e,'orcdchomp').SendCommand(
   'runchomp robot BarrettWAM adofgoal 7 0.5 1.0 0.0 2.0 0.0 -1.0 0.0')
t = openravepy.RaveCreateTrajectory(e,'').deserialize(t_data)
print 'Duration:', t.GetDuration()

raw_input('Press [Enter] to run the trajectory ...')
r.GetController().SetPath(t)

raw_input('Press [Enter] to quit.')
e.Destroy()
openravepy.RaveDestroy()
