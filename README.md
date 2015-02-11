# or_urdf

or_urdf is an OpenRAVE plugin for loading a URDF and, optionally, SRDF file as
an OpenRAVE `KinBody` or `Robot`. This package provides the OpenRAVE `URDF`
module through a plugin. This plugin can be instantiated using the following
Python code:

```python
module = RaveCreateModule(env, 'urdf')
```

Once the module has been instantiated, you can use the module to load a
`KinBody` by calling the custom `load` command with a path to a URDF file.
Similarly, you can create an OpenRAVE `Robot` calling the `load` command with
paths to both `URDF` and `SRDF` files.


## Why URDF and SRDF?

[URDF](http://wiki.ros.org/urdf) and [SRDF](http://wiki.ros.org/srdf) are the
standard XML file formats used to describe robots in [ROS](http://www.ros.org).
These files are [available for many robots](http://wiki.ros.org/urdf/Examples)
and are a key requirement of using
[MoveIt!](http://moveit.ros.org) and/or [RViz](http://wiki.ros.org/rviz). This
module uses the standard [urdfdom](https://github.com/ros/urdfdom) and
[srdfdom](https://github.com/ros-planning/srdfdom) parsers and, thus, should be
compatible with any robot specification that obeys the URDF and SRDF standards.


## Why programmatic construction?

or_urdf takes the unique approach of *progammatically constructing OpenRAVE
objects* from the URDF and SRDF files. This is in contrast to other conversion
functions, like the [`urdf_to_collada`
script](http://wiki.ros.org/collada_urdf), which converts the input URDF into
an intermediate file format. Programmatically constructing the OpenRAVE objects
has several key advantages over this alternative approach:

1. Relative `file://` and `package://` URI's are resolved at run-time.
2. There is no need to re-generate any auto-generated files when the URDF or
   SRDF files change.
3. There is no loss in precision due to the serialization and deserialization
   of floating point numbers.
4. The URDF and SRDF specifications can be loaded directly from the
   `robot_description` and `semantic_robot_description` ROS parameters.


## Loading a KinBody from URDF

The following code will load the URDF model `/path/to/my/model.urdf` as an
OpenRAVE `KinBody`:

```python
with env:
    name = module.SendCommand('load /path/to/my/model.urdf')
    body = env.GetKinBody(name)
```

The following OpenRAVE properties have no equivalent in URDF and, thus, must be
manually configured on a `KinBody` created by or_urdf:

- Robot DOF resolutions (`robot.SetDOFResolutions`)
- Mass and inertia properties are **currently untested**

The `load` command programmatically creates a `KinBody` from the URDF by
building a list of `LinkInfo` and `JointInfo` structures. This `KinBody` is
then added to the environment with the `anonymous` flag set. The command
returns the name of the object that was added to the environment. This is
necessary to avoid introducing name conflicts or ambiguity when loading
multiple instances of a URDF file into the same environment.

You can easily change the name of the resultant `KinBody` if the name generated
by this automatic procedure are undesirable. To do so, you must: (1) remove the
`KinBody` from the environment, (2) change the `KinBody`'s name, and (3) add
the `KinBody` back to the environment. For example:

```python
with env:
    name = module.SendCommand('load /path/to/my/model.urdf')
    body = env.GetKinBody(name)
    
    env.Remove(body)
    body.SetName('my_custom_name')
    env.Add(body)
```


## Loading a Robot from URDF and SRDF

or_urdf also supports loading an OpenRAVE `Robot` from the combination of a
URDF and SRDF file. In this case, the URDF file fills the role of an OpenRAVE
`.kinbody.xml` file and the SRDF file fills the role of an OpenRAVE
`.robot.xml` file. Unfortunately, there is not a direct mapping between the
SRDF file format and the features supported by OpenRAVE. or_urdf performs the
conversion as follows:

- `<disable_collisions>`: flags the pair of links ad adjacent
- `<link_sphere_approximation>`: is used to specify one or more `<sphere>` tags, which:
    - `<sphere>`: creates a sphere geometry in the `spheres` geometry group
- `<end_effector>`: defines a manipulator
    - `name`: sets the name of the OpenRAVE manipulator
    - `parent_group`: contains the manipulator DOFs (this is **required**)
    - `group`: contains the gripper DOFs (this is **required**, but may be empty)
- `<passive_joint>`: not used
- `<virtual_joint>`: not used
- `<group_state>`: not used
- `<group>`: not used, except if referenced in `<end_effector>`

The following OpenRAVE properties have no equivalent in SRDF and, thus, must be
manually configured on a `Robot` create by `or_urdf`:

- Robot DOF weights (`robot.SetDOFWeights`)
- Manipulator IK solver (`manipulaor.SetIkSolver`)
- Manipulator closing/chucking direction (`manipulaor.SetClosingDirection` or
  `manipulator.SetChuckingDirection`, depending upon the version of OpenRAVE)
- Manipulator tool transform and direction (`manipulator.SetLocalToolTransform`
  and `manipulator.SetLocalToolDirection`)
- All methods on `KinBody` listed above

Just as with creating a `KinBody`, or_urdf programmatically creates the `Robot`
by constructing `LinkInfo`, `JointInfo`, and `ManipulatorInfo` structs. The
following code creates an OpenRAVE `Robot` from the paired
`/path/to/my/model.urdf` and `/path/to/my/model.srdf` files:

```python
with env:
    name = module.SendCommand('load /path/to/my/model.urdf /path/to/my/model.srdf')
    body = env.GetRobot(name)
```

See above for more information about how to rename the robot created by the
module.


## License

or_urdf is licensed under a BSD license. See `LICENSE` for more information.


# Contributors

or_urdf was developed by the
[Personal Robotics Lab](https://personalrobotics.ri.cmu.edu) in the
[Robotics Institute](https://www.ri.cmu.edu) at
[Carnegie Mellon University](http://www.cmu.edu). This library is developed and
maintained by
[Michael Koval](https://github.com/mkoval) and
[Pras Velagapudi](https://github.com/psigen).

