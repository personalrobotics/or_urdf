#!/usr/bin/env python
#
# Loads a URDF to test the loader.
#
import openravepy

env = openravepy.Environment()
plugin = openravepy.RaveCreateModule(env, "URDFLoader")
plugin.SendCommand("load /homes/pkv/ros/local/systemconf/herb2.urdf")
#plugin.SendCommand("load /homes/pkv/ros/local/apps/librarian/tema_tim_description/robots/tim/tim.urdf")

