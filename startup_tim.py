#!/usr/bin/env python
#
# Loads a URDF to test the loader.
#
import os;

# Get this script path (in or_urdf) and add it to the openrave path
or_urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lib')
or_plugin_path = os.getenv('OPENRAVE_PLUGINS', '')
os.environ['OPENRAVE_PLUGINS'] = os.pathsep.join([or_urdf_path, or_plugin_path])

import openravepy

env = openravepy.Environment()
env.SetViewer('qtcoin')
plugin = openravepy.RaveCreateModule(env, "urdf")

#plugin.SendCommand("load /homes/pkv/ros/local/systemconf/herb2.urdf")
plugin.SendCommand("load /homes/pkv/ros/local/apps/librarian/tema_tim_description/robots/tim/tim.urdf")
#plugin.SendCommand("load /homes/pkv/ros/local/herb_urdf/robots/herb_urdf.URDF")
