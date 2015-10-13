^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package or_urdf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Added support for multiple collision objects within one link (`#13 <https://github.com/personalrobotics/or_urdf/issues/13>`_)
* Added missing dependency on OpenRAVE
* Set the source URI of loaded bodies (`#14 <https://github.com/personalrobotics/or_urdf/issues/14>`_)
* Contributors: Chris Dellin, Dawid Seredyński, Michael Koval

0.3.0 (2015-04-15)
------------------
* Update README.md
* Added some more unsupported features to the README
* Fixed a typo (thanks @cdellin!)
* Merge pull request `#12 <https://github.com/personalrobotics/or_urdf/issues/12>`_ from personalrobotics/bugfix/8
  Support package:// URDF and SRDF URIs
* Support package:// URDF and SRDF URIs (`#8 <https://github.com/personalrobotics/or_urdf/issues/8>`_)
* Merge pull request `#11 <https://github.com/personalrobotics/or_urdf/issues/11>`_ from personalrobotics/bugfix/ContinuousJointLimits
  Fixed support for continuous joints.
* Added a fix for limits on continuous joints.
* Don't convert revolute joints to continuous joints.
  This was previously happening if a continuous joint defined any limits,
  including velocity and effort limits.
* Force continuous joints to have +/-INF limits.
* Contributors: Michael Koval

0.2.0 (2015-03-30)
------------------
* Replaced RAVELOG_INFO with RAVELOG_DEBUG.
* Manually apply the URDF collision scale to meshes.
* Added support for scaling visual meshes.
* Added LICENSE.
* Added a README.
* Stripped out YAML dependencies (unnecessary if an SRDF is being used).
* Contributors: Michael Koval, Prasanna Velagapudi
