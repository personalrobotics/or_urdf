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

# Start openrave
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
e = openravepy.Environment()
e.SetViewer('qtcoin')

# Load the environment
e.Load('environments/intelkitchen_robotized_herb2.env.xml')

# Set up the robot, ik, active manip, active dofs
r = e.GetRobots()[0]
libprrave.ik.loadfor(r)
r.SetTransform(numpy.array(
   [[ 0., 1., 0.,  0.1880 ],
    [-1., 0., 0., -1.3748 ],
    [ 0., 0., 1.,  0.     ],
    [ 0., 0., 0.,  1.     ]]))
r.SetDOFValues(numpy.array(
   [  5.3, -1.8, -0.8,  2.4, -3.2,  0.0,  0.0952,  0.0, 0.0, 0.0, 0.0,
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
e.GetKinBody('bigbox').SetTransform(numpy.array(
   [[ 1., 0., 0., -0.5188 ],
    [ 0., 1., 0., -2.0258 ],
    [ 0., 0., 1.,  0.6005 ],
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



##create the TSR string
##place the first TSR's reference frame at the object's frame relative to world frame
#T0_w = bottle.GetTransform()
##get the TSR's offset frame in w coordinates    
#Tw_e = MakeTransform(rodrigues([0,pi,0])*rodrigues([pi / 2, 0, 0]), mat([0, 0.20, 0.1]).T)
##Tw_e = MakeTransform(rodrigues([pi / 2, 0, 0]), mat([0, 0.22, 0.1]).T)
##define bounds to only allow rotation of the hand about z axis and a small deviation in translation along the z axis
#Bw = mat([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -pi, pi])
#TSRstring = SerializeTSR(0, 'NULL', T0_w, Tw_e, Bw) 


#if len(sys.argv)==1:
  

    #chomp_darpa = RaveCreateProblem(env,'chomp_darpa')
    #env.LoadProblem(chomp_darpa, '')


    ##resp=chomp_darpa.SendCommand('plan robot BarrettWAM configuration '+goal+' trajfile chomp_darpa_traj.txt')
    ##resp=chomp_darpa.SendCommand('plan robot BarrettWAM eetrans' +SerializeTransform(goal_trans)+ 'trajfile chomp_darpa_traj.txt')
    #resp=chomp_darpa.SendCommand('plan robot BarrettWAM TSR ' + TSRstring+ ' trajfile chomp_darpa_traj.txt')
    
    ##raw_input("Press ENTER for trajectory.")
    #if (int(resp)==1):
        #resp = chomp_darpa.SendCommand('execute BarrettWAM chomp_darpa_traj.txt')
    #else:
        #print "Sorry, CHOMP failed this time."    
    
    
    
#elif sys.argv[1]=="oldchomp":
        
    #chomp = RaveCreateProblem(env,'chomp')
    #env.LoadProblem(chomp, '')
    

    #resultsfile = "testing_results.txt"
    #trajfile = "chomp_traj.txt"
    #command = 'goto BarrettWAM '+'1 ' + ' ' + goal +' ' + resultsfile +  ' ' + trajfile + ' TrajectoryTracking.txt '+ 'hard'+ ' '+'nohmc'
    #resp = chomp.SendCommand(command)

    #raw_input("ENTER for traj")
    #command = 'execute BarrettWAM '+trajfile
    #chomp.SendCommand(command)   

#elif sys.argv[1]=="vis":
    #end_points, num_waypoints, num_trajs = GetEndEffectorTrajectories()
    
    #balls=[]
    #for i in range(num_waypoints):
        #ball=env.ReadKinBodyXMLFile('smallsphere.kinbody.xml')
        #ball.SetName(str(i))
        #env.AddKinBody(ball)
        #balls.append(ball)
        
    #raw_input("press enter to see traj")    
    #for i in range(num_trajs):
        #for j in range(num_waypoints):
            ##put in the ball
            #t=end_points[i*int(num_waypoints[0])+j]
            #balls[j].SetTransform(t)
        #sleep(0.08)  
            
    
        
    
        
    
