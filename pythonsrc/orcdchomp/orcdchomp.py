import openravepy

def runchomp(mod, robot=None, adofgoal=None, n_iter=None, lambda_=None):
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
   traj_data = mod.SendCommand(cmd)
   return openravepy.RaveCreateTrajectory(mod.GetEnv(),'').deserialize(traj_data)
