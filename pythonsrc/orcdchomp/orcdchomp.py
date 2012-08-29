# \file orcdchomp.py
# \brief Python interface to orcdchomp.
# \author Christopher Dellin
# \date 2012

# (C) Copyright 2012 Carnegie Mellon University

import types
import openravepy

def bind(mod):
   mod.viewspheres = types.MethodType(viewspheres,mod)
   mod.computedistancefield = types.MethodType(computedistancefield,mod)
   mod.runchomp = types.MethodType(runchomp,mod)

def shquot(s):
   return "'" + s.replace("'","'\\''") + "'"
   
def viewspheres(mod, robot=None):
   cmd = 'viewspheres'
   if robot is not None:
      if hasattr(robot,'GetName'):
         cmd += ' robot %s' % shquot(robot.GetName())
      else:
         cmd += ' robot %s' % shquot(robot)
   print 'cmd:', cmd
   return mod.SendCommand(cmd)

def computedistancefield(mod, kinbody=None, cube_extent=None, aabb_padding=None, cache_filename=None):
   cmd = 'computedistancefield'
   if kinbody is not None:
      if hasattr(kinbody,'GetName'):
         cmd += ' kinbody %s' % shquot(kinbody.GetName())
      else:
         cmd += ' kinbody %s' % shquot(kinbody)
   if cube_extent is not None:
      cmd += ' cube_extent %f' % cube_extent
   if aabb_padding is not None:
      cmd += ' aabb_padding %f' % aabb_padding
   if cache_filename is not None:
      cmd += ' cache_filename %s' % shquot(cache_filename)
   print 'cmd:', cmd
   return mod.SendCommand(cmd)

def runchomp(mod, robot=None, adofgoal=None, n_iter=None, max_time=None,
   lambda_=None, starttraj=None, n_points=None, start_tsr=None,
   use_momentum=None, use_hmc=None, hmc_resample_lambda=None, seed=None,
   epsilon=None, epsilon_self=None, obs_factor=None, obs_factor_self=None,
   no_collision_exception=None, no_report_cost=None,
   dat_filename=None, trajs_fileformstr=None):
   cmd = 'runchomp'
   if robot is not None:
      if hasattr(robot,'GetName'):
         cmd += ' robot %s' % shquot(robot.GetName())
      else:
         cmd += ' robot %s' % shquot(robot)
   if adofgoal is not None:
      cmd += ' adofgoal %s' % shquot(' '.join([str(v) for v in adofgoal]))
   if n_iter is not None:
      cmd += ' n_iter %d' % n_iter
   if max_time is not None:
      cmd += ' max_time %f' % max_time
   if lambda_ is not None:
      cmd += ' lambda %0.04f' % lambda_
   if starttraj is not None:
      in_traj_data = starttraj.serialize(0) # options
      cmd += ' starttraj %s' % shquot(in_traj_data)
   if n_points is not None:
      cmd += ' n_points %d' % n_points
   if start_tsr is not None:
      cmd += ' start_tsr \'%s\'' % start_tsr.serialize()
   if use_momentum is not None and use_momentum:
      cmd += ' use_momentum'
   if use_hmc is not None and use_hmc:
      cmd += ' use_hmc'
   if hmc_resample_lambda is not None:
      cmd += ' hmc_resample_lambda %f' % hmc_resample_lambda
   if seed is not None:
      cmd += ' seed %d' % seed
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
   if dat_filename is not None:
      cmd += ' dat_filename %s' % shquot(dat_filename)
   if trajs_fileformstr is not None:
      cmd += ' trajs_fileformstr %s' % shquot(trajs_fileformstr)
   print 'cmd:', cmd
   out_traj_data = mod.SendCommand(cmd)
   return openravepy.RaveCreateTrajectory(mod.GetEnv(),'').deserialize(out_traj_data)
