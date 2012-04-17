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

# table
e.Load('objects/furniture/table2.kinbody.xml')
e.GetKinBody('table').SetTransform(numpy.array(
   [[ 0.,-1., 0., 0.   ],
    [ 1., 0., 0., 0.   ],
    [ 0., 0., 1., 0.75 ],
    [ 0., 0., 0., 1.   ]]))

# bottle
e.Load('objects/household/fuze_bottle.kinbody.xml')
e.GetKinBody('fuze_bottle').SetTransform(numpy.array(
   [[ 1., 0., 0., 0.   ],
    [ 0., 1., 0., 0.   ],
    [ 0., 0., 1., 0.77 ],
    [ 0., 0., 0., 1.   ]]))
Tw_pa_righty = numpy.array(
   [[ 0, 0, 1,-0.10  ],
    [-1, 0, 0, 0.    ],
    [ 0,-1, 0, 0.075 ],
    [ 0, 0, 0, 1.    ]])

# robot
e.Load('robots/barrettwam.robot.xml')
r = e.GetRobot('BarrettWAM')
r.SetTransform(numpy.array(
   [[ 0., 0., 1.,-1.0 ],
    [ 0., 1., 0., 0.  ],
    [-1., 0., 0., 1.0 ],
    [ 0., 0., 0., 1. ]]))

# Create the module
libprrave.rave.load_module(e, 'orcdchomp', 'blah_load_string')

# set up active manip, active dofs
libprrave.ik.loadfor(r)
r.SetActiveManipulator('arm')
m = r.GetActiveManipulator()
r.SetActiveDOFs(m.GetArmIndices())
T_pa_ee = numpy.array(
   [[ 1., 0., 0., 0. ],
    [ 0., 1., 0., 0.  ],
    [ 0., 0., 1., 0.064 ],
    [ 0., 0., 0., 1. ]])


# get IK solution for bottle
Tee = numpy.dot(
   numpy.dot(
      e.GetKinBody('fuze_bottle').GetTransform(),
      numpy.array([[ 1., 0., 0., 0. ],
                   [ 0., 1., 0., 0. ],
                   [ 0., 0., 1., 0. ],
                   [ 0., 0., 0., 1. ]])),
   numpy.dot(Tw_pa_righty, T_pa_ee)
)
q_goal = m.FindIKSolution(Tee, 0)

r.SetActiveDOFValues([2.5,-1.8,0.0,2.0,0.0,0.2,0.0])
#r.SetActiveDOFValues(q_goal)

e.Load('objects/misc/coordframe.kinbody.xml')
frame = e.GetKinBody('coordframe')
#frame.SetTransform(numpy.dot(m.GetEndEffectorTransform(),numpy.linalg.inv(T_pa_ee)))

# Disable the robot from collision checking while computing distance field 
raw_input('Press [Enter] compute distance field ...')
r.Enable(False)
libprrave.rave.get_module(e,'orcdchomp').SendCommand(
   'computedistancefield robot BarrettWAM')
r.Enable(True)

raw_input('Press [Enter] run chomp ...')
t_data = libprrave.rave.get_module(e,'orcdchomp').SendCommand(
   'runchomp robot BarrettWAM adofgoal 7 %s' % (' '.join([str(v) for v in q_goal])))
t = openravepy.RaveCreateTrajectory(e,'').deserialize(t_data)

try:
   while True:
      raw_input('Press [Enter] to run the trajectory ...')
      with e:
         r.GetController().SetPath(t)
except KeyboardInterrupt:
   print

e.Destroy()
openravepy.RaveDestroy()
