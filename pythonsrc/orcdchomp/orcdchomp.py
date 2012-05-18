import types
import openravepy

def bind(mod):
   mod.viewspheres = types.MethodType(viewspheres,mod)
   mod.computedistancefield = types.MethodType(computedistancefield,mod)
   mod.runchomp = types.MethodType(runchomp,mod)
   
def viewspheres(mod, robot=None):
   cmd = 'viewspheres'
   if robot is not None:
      if hasattr(robot,'GetName'):
         cmd += ' robot %s' % robot.GetName()
      else:
         cmd += ' robot %s' % robot
   return mod.SendCommand(cmd)

def computedistancefield(mod, robot=None):
   cmd = 'computedistancefield'
   if robot is not None:
      if hasattr(robot,'GetName'):
         cmd += ' robot %s' % robot.GetName()
      else:
         cmd += ' robot %s' % robot
   return mod.SendCommand(cmd)

def runchomp(mod, robot=None, adofgoal=None, n_iter=None, lambda_=None, starttraj=None):
   cmd = 'runchomp'
   if robot is not None:
      if hasattr(robot,'GetName'):
         cmd += ' robot %s' % robot.GetName()
      else:
         cmd += ' robot %s' % robot
   if adofgoal is not None:
      cmd += ' adofgoal %d %s' % (len(adofgoal), ' '.join([str(v) for v in adofgoal]))
   if n_iter is not None:
      cmd += ' n_iter %d' % n_iter
   if lambda_ is not None:
      cmd += ' lambda %0.04f' % lambda_
   if starttraj is not None:
      in_traj_data = starttraj.serialize(0) # options
      cmd += ' starttraj %d %s' % (len(in_traj_data), in_traj_data)
   out_traj_data = mod.SendCommand(cmd)
   return openravepy.RaveCreateTrajectory(mod.GetEnv(),'').deserialize(out_traj_data)
