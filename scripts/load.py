#!/usr/bin/env python
PACKAGE = 'or_urdf'
import argparse, openravepy, os, sys

parser = argparse.ArgumentParser(description='loads a URDF model into OpenRAVE')
parser.add_argument('urdf_path', type=str)
parser.add_argument('srdf_path', type=str, default='', nargs='?')
parser.add_argument('-i', '--interactive', action='store_true', help='display the model in an OpenRAVE viewer')
parser.add_argument('-c', '--config', type=str, action='store', help='config file for urdf generation')
args = parser.parse_args()

# Load the plugin.
env = openravepy.Environment()
plugin = openravepy.RaveCreateModule(env, 'urdf')
if plugin is None:
    parser.error('Unable to load urdf plugin.')
    sys.exit(1)

# Generate the KinBody XML.
try:
    kinbody_xml = plugin.SendCommand('load {0:s} {1:s}'.format(args.urdf_path, args.srdf_path))
    if kinbody_xml is None:
        raise openravepy.openrave_exception('An unknown error has occurred.')
except openravepy.openrave_exception, e:
    parser.error('Failed generating KinBody: {0:s}'.format(e.message))
    sys.exit(1)

#env.SetViewer('qtcoin')

body = env.GetBodies()[0]
handles = list()

"""
for link in body.GetLinks():
    pose = link.GetTransform()
    handle = openravepy.misc.DrawAxes(env, pose, 0.2, 2)
    handles.append(handle)
"""

if args.interactive:
    import IPython; IPython.embed()
