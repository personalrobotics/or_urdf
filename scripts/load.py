#!/usr/bin/env python
PACKAGE = 'or_urdf'
import argparse, openravepy, os, rospkg, sys
ros_pack = rospkg.RosPack()

parser = argparse.ArgumentParser(description='loads a URDF model into OpenRAVE')
parser.add_argument('input_path', type=str, help='path to the URDF file')
parser.add_argument('output_path', type=str, nargs='?', help='output path for the KinBody file')
parser.add_argument('-i', '--interactive', action='store_true', help='display the model in an OpenRAVE viewer')
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
    kinbody_xml = plugin.SendCommand('load {0:s}'.format(args.input_path))
    if kinbody_xml is None:
        raise openravepy.openrave_exception('An unknown error has occurred.')
except openravepy.openrave_exception, e:
    parser.error('Failed generating KinBody: {0:s}'.format(e.message))
    sys.exit(1)

if args.output_path is not None:
    with open(args.output_path, 'wb') as output_stream:
        output_stream.write(kinbody_xml)

    print 'Saved KinBody to {0:s}'.format(args.output_path)

if args.interactive:
    env.LoadData(kinbody_xml)
    env.SetViewer('qtcoin')

    try:
        from IPython.Shell import IPShellEmbed
        shell = IPShellEmbed()
        shell(local_ns=locals())
    # Just wait for a Control+C if IPython is not available.
    except ImportError:
        try:
            while True: pass
        except KeyboardInterrupt:
            pass

env.Destroy()
openravepy.RaveDestroy()
