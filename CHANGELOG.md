# Changelog

## Version 0.0.52
- `_parse_mass()` returns `float` instead of `str`
- default parsed mass is 0.0 instead of 1.0 (see http://wiki.ros.org/urdf/XML/link)
- Update `trimesh` dependency to `trimesh[easy]` (to support loading Collada meshes)
- Won't crash when loading model with joints that mimic unactuated/fixed joints.

## Version 0.0.51
- Fix path separator issues in Windows [Issue 27](https://github.com/clemense/yourdfpy/issues/27)

## Version 0.0.50
- A nothingburger

## Version 0.0.49
- Fix single link URDF trimesh scene bug [PR26](https://github.com/clemense/yourdfpy/pull/26)
## Version 0.0.48
- Implement `-c`/`--configuration` argument for `yourdfpy`
- Add `--animate` flag to `yourdfpy`

## Version 0.0.47
- Bugfix: Parsing box dimensions
- Change to `trimesh.primitives.*` for geometric primitives (instead of `trimesh.creation.*`)

## Version 0.0.46
- Bugfix: Named material with color wouldn't be applied

## Version 0.0.45
- Upgrade to trimesh version 3.11.2
- Add `__eq__` operator to URDF based on equality of individual elements (order-invariant) [PR18](https://github.com/clemense/yourdfpy/pull/18)
- Add material information [PR15](https://github.com/clemense/yourdfpy/pull/15)
- Improve mesh filename search [PR14](https://github.com/clemense/yourdfpy/pull/14)

## Version 0.0.44
- Parse and write `name` attribute of `material` element 
- Apply colors to mesh if available
- Handle empty scene exception (in case of URDF without meshes)

## Version 0.0.43
- Skip material loading for collision geometries

## Version 0.0.42
- Fix bug when updating robots with mimic joints 

## Version 0.0.41
- yourdfpy.Sphere visible

## Version 0.0.40
- Add 'file_path', 'file_name', and 'file_element' to trimesh geometry's metadata in case loaded OBJ file contains multiple parts
- Load collision geometry for viz

## Version 0.0.39
- Fix mimic joint issue

## Version 0.0.38
- Change namespace of filename handlers

## Version 0.0.37
- Use visual/collision name property as geometry name for scene graph
- Write visual/collision name property to URDF

## Version 0.0.36
- Fix validation of JointLimit
- Add Dynamics to __init__.py
 
## Version 0.0.35
- Add `force_single_geometry_per_link` feature: similar to `collision_mesh` in urdfpy; will concatenate all meshes in a single link and only create one node in the scene graph. This is the new default for loading the collision scene.

## Version 0.0.34
- Fix missing `Collision` exposure in init.py
- Add `force_collision_mesh`

## Version 0.0.33
- Add `force_mesh` to constructor; allows loading mesh files as single meshes instead of turning them into graphs (since trimesh can't deal with meshes with multiple textures)

## Version 0.0.32
- Fix `continuous` joint during forward kinematics
- Introduce DOF indices for actuated joints (to handle planar and floating types)

## Version 0.0.31
- Add `num_dofs`
- Add `zero_cfg` property
- Change function `get_default_cfg` to property `center_cfg`
- Initial configuration is now the zero configuration not the center configuration (as previously)

## Version 0.0.30
## Version 0.0.29
## Version 0.0.28
## Version 0.0.27
- Bugfix in travis deployment pipeline

## Version 0.0.26
- Bugfix: rename `generate_scene_graph` parameter
- Bugfix of bugfix of previous version, which introduced a new bug
- Bugfix: root URDF result of `split_along_joints`  (scene was not in sync with model)
- Add params to `split_along_joints`
- Bugfix: `parse_inertia` resulted in wrong matrix dtype

## Version 0.0.25
- Bugfix: `get_default_cfg` returns flattened array

## Version 0.0.24
- Added pytests
- Separate visual and collision scene
- Rename constructor's parameter `create_scene_graph` to `build_scene_graph`
- Added ROS validation rules
- Rename `update_trimesh_scene` to `update_cfg`, change arguments
- Add `get_transform` function
- Rename `get_default_configuration` to `get_default_cfg`
- Proper handling of `mimic` joints
- New members for `actuated_joints`
- New `base_link` property

## Version 0.0.23
- The Great YOURDFPY Steering Committee (G.Y.S.C.) decides to jump as many version numbers ahead as needed to pass urdfpy

## Version 0.0.14
- The Great YOURDFPY Steering Committee (G.Y.S.C.) gives up on using only version numbers that are prime

## Version 0.0.13
- Adding images. For the Github crowd.

## Version 0.0.11
- These numbers are going up quickly.

## Version 0.0.7
- Wow. This was quite the evening.

## Version 0.0.5
- The Great YOURDFPY Steering Committee (G.Y.S.C.) decides to only use version numbers that are prime

## Version 0.0.3
- A version few remember and many ignored

## Version 0.0.1
- A version nobody remembers
