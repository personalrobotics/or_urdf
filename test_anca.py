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

import cbirrt2.cbirrt_problem as cprob

# Start openrave
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
e = openravepy.Environment()

# Load the environment
e.Load('environments/intelkitchen_robotized_herb2.env.xml')
e.SetViewer('qtcoin')

# Set up the robot, ik, active manip, active dofs
with e:
   r = e.GetRobots()[0]
   libprrave.ik.loadfor(r)
   r.SetTransform(numpy.array(
      [[ 0., 1., 0.,  0.1880 ],
       [-1., 0., 0., -1.3748 ],
       [ 0., 0., 1.,  0.     ],
       [ 0., 0., 0.,  1.     ]]))
   #r.SetDOFValues(numpy.array(
   #   [  5.3, -1.8, -0.8,  2.4, -3.2,  0.0,  0.0952,  0.0, 0.0, 0.0, 0.0,
   #      0.9, -1.9,  0.7,  2.4, -1.6,  0.0,  0.8,     1.6, 1.6, 1.6, 0.0  ]))
   r.SetDOFValues(numpy.array(
      [  5.3, -1.5,  2.9,  2.4, -3.2,  0.0,  0.0952,  0.0, 0.0, 0.0, 0.0,
         0.9, -1.9,  0.7,  2.4, -1.6,  0.0,  0.8,     1.6, 1.6, 1.6, 0.0  ]))
   r.SetActiveManipulator('right_wam')
   m = r.GetActiveManipulator()
   r.SetActiveDOFs(m.GetArmIndices())

# Load the bottle
e.Load('objects/household/fuze_bottle.kinbody.xml')
e.GetKinBody('fuze_bottle').SetTransform(numpy.array(
   [[ 1., 0., 0.,  0.0424 ],
    [ 0., 1., 0., -2.1717 ],
    [ 0., 0., 1.,  0.7912 ],
    [ 0., 0., 0.,  1.     ]]))

# Load the box
e.Load('objects/misc/bigbox.kinbody.xml')
#e.GetKinBody('bigbox').SetTransform(numpy.array(
#   [[ 1., 0., 0., -0.5188 ],
#    [ 0., 1., 0., -2.0258 ],
#    [ 0., 0., 1.,  0.6005 ],
#    [ 0., 0., 0.,  1.     ]]))
e.GetKinBody('bigbox').SetTransform(numpy.array(
   [[ 1., 0., 0., -0.4188 ],
    [ 0., 1., 0., -1.4258 ],
    [ 0., 0., 1.,  1.7005 ],
    [ 0., 0., 0.,  1.     ]]))

# Load the rice pilaf
e.Load('objects/household/ricepilaf.kinbody.xml')
theta = -0.4685
e.GetKinBody('ricepilaf').SetTransform(numpy.array(
   [[ math.cos(theta), -math.sin(theta), 0., -0.1607 ],
    [ math.sin(theta),  math.cos(theta), 0., -2.0936 ],
    [              0.,               0., 1.,  0.9171 ],
    [              0.,               0., 0.,  1.     ]]))

# Set the goal ...
q_goal = [3.30768, -0.631765, -0.113948, 2.15259, -3.59911, 1.5041, 1.51]
#goal = '7 3.30768 -0.631765 -0.113948 2.15259 -3.59911 1.5041 1.51'
#goal = '7 3.30768 -0.631765 -0.113948 1.06 -3.59911 1.5041 1.51'
#goal_trans = numpy.array(
#   [[ -0.92481091, -0.01074582,  0.38027531, -0.03772895 ],
#    [ -0.38020306, -0.00819394, -0.92486674, -1.98194454 ],
#    [  0.0130544 , -0.99990869,  0.00349225,  0.89187527 ],
#    [  0.        ,  0.        ,  0.        ,  1.         ]])

# Create the module
libprrave.rave.load_module(e, 'orcdchomp', 'blah_load_string')
libprrave.rave.load_module(e, 'cbirrt', r.GetName())

raw_input('Press [Enter] to view spheres ...')
libprrave.rave.get_module(e,'orcdchomp').SendCommand(
   'viewspheres robot %s' % r.GetName())
raw_input('Press [Enter] to clear spheres ...')
with e:
   for b in e.GetBodies():
      if b.GetName().startswith('orcdchomp_sphere_'):
         e.Remove(b)

# Disable the robot from collision checking while computing distance field 
raw_input('Press [Enter] compute distance field ...')
r.Enable(False)
libprrave.rave.get_module(e,'orcdchomp').SendCommand(
   'computedistancefield robot BarrettWAM')
r.Enable(True)

try:
   raw_input('Press [Enter] run chomp ...')
   t_data = libprrave.rave.get_module(e,'orcdchomp').SendCommand(
      'runchomp robot BarrettWAM adofgoal 7 %s lambda 50.0 n_iter 50' % (' '.join([str(v) for v in q_goal])))
   t = openravepy.RaveCreateTrajectory(e,'').deserialize(t_data)
except RuntimeError as ex:
   print ex
   t = None

try:
   while t is not None:
      raw_input('Press [Enter] to run the trajectory ...')
      with e:
         r.GetController().SetPath(t)
except KeyboardInterrupt:
   print

raw_input('Press [Enter] to quit ...')
e.Destroy()
openravepy.RaveDestroy()
