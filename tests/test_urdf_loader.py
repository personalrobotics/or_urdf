#!//bin/env python

import numpy
import unittest
import openravepy
import subprocess
import os
import sys

# Find the bh_280 URDF file
from catkin.find_in_workspaces import find_in_workspaces
share_directories = find_in_workspaces(search_dirs=['share'],
								project='herb_description')
if not share_directories:
    logger.error('Unable to find the BH280 model. Do you have the'
                 ' package herb_description installed?')
    raise ValueError('Unable to find BH280 model.')

found_models = False
for share_directory in share_directories:
	urdf_path = os.path.join(share_directories[0], 'robots', 'bh280.urdf')
	if os.path.exists(urdf_path):
		found_models=True
		break

if not found_models:
    logger.error('Missing URDF file for BH280.'
                 ' Is the herb_description package properly installed?')
    raise ValueError('Unable to find BH280 URDf file')

#Create module for loading URDF


class TestORUrdf(unittest.TestCase):

	def setUp(self):

		env = openravepy.Environment()
		urdf_module = openravepy.RaveCreateModule(env, 'urdf')
		self.env = env

		with self.env:
			#Load the bh280 URDF model
			print urdf_path
			arg = 'load {:s}'.format(urdf_path)
			bh_name = urdf_module.SendCommand(arg)
			self.bh_body = env.GetKinBody(bh_name)

	def tearDown(self):
		self.env.Destroy()


	def TestLinkLocalInertia(self):

		#Test inertia for a random link
		hand_base_link = self.bh_body.GetLink('/hand_base')
		numpy.testing.assert_array_almost_equal(
						hand_base_link.GetLocalInertia(),
						numpy.diag([0.0006986,0.00050354,0.00062253]))

	def TestLinkMass(self):

		#Test mass for a random link
		finger0_0 = self.bh_body.GetLink('/finger0_0')
		numpy.testing.assert_almost_equal(finger0_0.GetMass(),0.14109)

	def TestLinkLocalCOM(self):

		finger0_0 = self.bh_body.GetLink('/finger0_0')
		numpy.testing.assert_array_almost_equal(
						finger0_0.GetLocalCOM(),
						numpy.array([0.030616,-7.3219e-05,-0.011201]))

	def TestJointLimits(self):

		#Test extent and velocity limits of a join
		joint = self.bh_body.GetJoint('/j00')
		numpy.testing.assert_array_almost_equal(
						joint.GetLimits()[0],numpy.array([0]))
		numpy.testing.assert_array_almost_equal(
						joint.GetLimits()[1],numpy.array([numpy.pi]))
		numpy.testing.assert_array_almost_equal(
						joint.GetMaxVel(),2.0)


	def TestParentChild(self):

		#Test parent and child links of joint
		joint = self.bh_body.GetJoint('/j01')
		numpy.testing.assert_equal(
			joint.GetHierarchyChildLink().GetName(),'/finger0_1')
		numpy.testing.assert_equal(
			joint.GetHierarchyParentLink().GetName(),'/finger0_0')

	def TestMimicEquation(self):

		#Test mimic equation for a join
		joint = self.bh_body.GetJoint('/j22')
		numpy.testing.assert_equal(
			joint.GetMimicEquation(),'/j21*0.321429+0.000000')
	

