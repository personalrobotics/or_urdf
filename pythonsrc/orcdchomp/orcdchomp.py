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

def computedistancefield(mod, kinbody=None, cube_extent=None, cache_filename=None):
   cmd = 'computedistancefield'
   if kinbody is not None:
      if hasattr(kinbody,'GetName'):
         cmd += ' kinbody %s' % kinbody.GetName()
      else:
         cmd += ' kinbody %s' % kinbody
   if cube_extent is not None:
      cmd += ' cube_extent %f' % cube_extent
   if cache_filename is not None:
      cmd += ' cache_filename %s' % cache_filename
   return mod.SendCommand(cmd)

def runchomp(mod, robot=None, adofgoal=None, n_iter=None, lambda_=None, starttraj=None, n_intpoints=None,
   epsilon=None, epsilon_self=None, obs_factor=None, obs_factor_self=None,
   no_collision_exception=None, no_report_cost=None):
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
   if n_intpoints is not None:
      cmd += ' n_intpoints %d' % n_intpoints
   if epsilon is not None:
      cmd += ' epsilon %f' % epsilon
   if epsilon_self is not None:
      cmd += ' epsilon_self %f' % epsilon_self
   if obs_factor is not None:
      cmd += ' obs_factor %f' % obs_factor
   if obs_factor_self is not None:
      cmd += ' obs_factor_self %f' % obs_factor_self
   if no_collision_exception is not None and no_collision_exception:
      cmd += ' no_collision_exception'
   if no_report_cost is not None and no_report_cost:
      cmd += ' no_report_cost'
   print 'cmd:', cmd
   out_traj_data = mod.SendCommand(cmd)
   return openravepy.RaveCreateTrajectory(mod.GetEnv(),'').deserialize(out_traj_data)
