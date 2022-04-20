# Changelog

## Version 0.0.44 (development)
- Parse and write `name` attribute of `material` element 
- Apply colors to mesh if available
- Handle empty scene exception (in case of URDF without meshes)

## Version 0.0.43 (development)
- Skip material loading for collision geometries

## Version 0.0.42 (development)
- Fix bug when updating robots with mimic joints 

## Version 0.0.41 (development)
- yourdfpy.Sphere visible

## Version 0.0.40 (development)
- Add 'file_path', 'file_name', and 'file_element' to trimesh geometry's metadata in case loaded OBJ file contains multiple parts
- Load collision geometry for viz

## Version 0.0.39 (development)
- Fix mimic joint issue

## Version 0.0.38 (development)
- Change namespace of filename handlers

## Version 0.0.37 (development)
- Use visual/collision name property as geometry name for scene graph
- Write visual/collision name property to URDF

## Version 0.0.36 (development)
- Fix validation of JointLimit
- Add Dynamics to __init__.py
 
## Version 0.0.35 (development)
- Add `force_single_geometry_per_link` feature: similar to `collision_mesh` in urdfpy; will concatenate all meshes in a single link and only create one node in the scene graph. This is the new default for loading the collision scene.

## Version 0.0.34 (development)
- Fix missing `Collision` exposure in init.py
- Add `force_collision_mesh`

## Version 0.0.33 (development)
- Add `force_mesh` to constructor; allows loading mesh files as single meshes instead of turning them into graphs (since trimesh can't deal with meshes with multiple textures)

## Version 0.0.32 (development)
- Fix `continuous` joint during forward kinematics
- Introduce DOF indices for actuated joints (to handle planar and floating types)

## Version 0.0.31 (development)
- Add `num_dofs`
- Add `zero_cfg` property
- Change function `get_default_cfg` to property `center_cfg`
- Initial configuration is now the zero configuration not the center configuration (as previously)

## Version 0.0.30 (development)
## Version 0.0.29 (development)
## Version 0.0.28 (development)
## Version 0.0.27 (development)
- Bugfix in travis deployment pipeline

## Version 0.0.26 (development)
- Bugfix: rename `generate_scene_graph` parameter
- Bugfix of bugfix of previous version, which introduced a new bug
- Bugfix: root URDF result of `split_along_joints`  (scene was not in sync with model)
- Add params to `split_along_joints`
- Bugfix: `parse_inertia` resulted in wrong matrix dtype

## Version 0.0.25 (development)
- Bugfix: `get_default_cfg` returns flattened array

## Version 0.0.24 (development)
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

## Version 0.0.23 (development)
- The Great YOURDFPY Steering Committee (G.Y.S.C.) decides to jump as many version numbers ahead as needed to pass urdfpy

## Version 0.0.14 (development)
- The Great YOURDFPY Steering Committee (G.Y.S.C.) gives up on using only version numbers that are prime

## Version 0.0.13 (development)
- Adding images. For the Github crowd.

## Version 0.0.11 (development)
- These numbers are going up quickly.

## Version 0.0.7 (development)
- Wow. This was quite the evening.

## Version 0.0.5 (development)
- The Great YOURDFPY Steering Committee (G.Y.S.C.) decides to only use version numbers that are prime

## Version 0.0.3 (development)
- A version few remember and many ignored

## Version 0.0.1 (development)
- A version nobody remembers
