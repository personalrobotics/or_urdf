#!/usr/bin/env python
PACKAGE = 'or_urdf'
import roslib; roslib.load_manifest(PACKAGE)
import argparse, openravepy, os, rospkg, sys
ros_pack = rospkg.RosPack()

parser = argparse.ArgumentParser(description='loads a URDF model into OpenRAVE')
parser.add_argument('urdf_path', type=str)
parser.add_argument('srdf_path', type=str, default='', nargs='?')
parser.add_argument('-i', '--interactive', action='store_true', help='display the model in an OpenRAVE viewer')
parser.add_argument('-c', '--config', type=str, action='store', help='config file for urdf generation')
args = parser.parse_args()

# Add the or_urdf plugin to the OPENRAVE_PLUGINS path.
or_urdf_path = ros_pack.get_path(PACKAGE)
or_urdf_plugins_path = os.path.join(or_urdf_path, 'lib')
or_plugins_path = os.getenv('OPENRAVE_PLUGINS', '')
os.environ['OPENRAVE_PLUGINS'] = os.pathsep.join([ or_urdf_plugins_path, or_plugins_path ])

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

env.SetViewer('qtcoin')

body = env.GetBodies()[0]
handles = list()

for link in body.GetLinks():
    pose = link.GetTransform()
    handle = openravepy.misc.DrawAxes(env, pose, 0.2, 2)
    handles.append(handle)
