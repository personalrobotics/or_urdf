#!/usr/bin/env python
#
# Loads a URDF to test the loader.
#
import os;
os.environ['OPENRAVE_PLUGINS'] = os.path.join(os.getcwd(), 'lib') + ':' + os.environ['OPENRAVE_PLUGINS']

import openravepy

env = openravepy.Environment()
env.SetViewer('qtcoin')
plugin = openravepy.RaveCreateModule(env, "urdf")

#plugin.SendCommand("load /homes/pkv/ros/local/systemconf/herb2.urdf")
plugin.SendCommand("load /homes/pkv/ros/local/apps/librarian/tema_tim_description/robots/tim/tim.urdf")
#plugin.SendCommand("load /homes/pkv/ros/local/herb_urdf/robots/herb_urdf.URDF")
