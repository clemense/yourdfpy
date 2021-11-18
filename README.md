# yourdfpy

Yet anOther URDF parser for Python. Yup, it's another one. Deal with it.

Yourdfpy is a simpler and easier-to-use library for loading, manipulating, validating, saving, and visualizing URDF files.

## Installation

You can install yourdfpy directly from pip:
```
pip install yourdfpy
```


## But why another one?!?
![Example URDFs](https://github.com/clemense/yourdfpy/blob/main/docs/_static/urdf_examples.jpg?raw=true)

|                                                                                          | [urdfpy](https://github.com/mmatl/urdfpy) | [urdf_parser_py](https://github.com/ros/urdf_parser_py) |    **yourdfpy**    |
| ---------------------------------------------------------------------------------------: | :---------------------------------------: | :-----------------------------------------------------: | :----------------: |
|                                                         Decouple parsing from validation |                                           |                                                         | :heavy_check_mark: |
|                                                     Decouple parsing from loading meshes |                                           |                   :heavy_check_mark:                    | :heavy_check_mark: |
|                                                                           Visualize URDF |            :heavy_check_mark:             |                                                         | :heavy_check_mark: |
|                                                                       Forward Kinematics |            :heavy_check_mark:             |                                                         | :heavy_check_mark: |
| Robustness Test on 12 URDF files from [here](https://github.com/ankurhanda/robot-assets) |                   4/12                    |                          6/12                           |       12/12        |
|                                             Avg. loading time per file (w/ mesh loading) |                  480 ms                   |                                                         |       370 ms       |
|                                                                       (w/o mesh loading) |                                           |                         3.2 ms                          |       6.2 ms       |
|                                          Test on 4 URDF files on which `urdfpy` succeeds |                 347.5 ms                  |                                                         |       203 ms       |
|                                  Test on 6 URDF files on which `urdf_parser_py` succeeds |                                           |                         2.6 ms                          |       3.8 ms       |

Entries for the above table were produced with:
```
robot_assets = ['robot-assets/urdfs/robots/barret_hand/bhand_model.URDF', 'robot-assets/urdfs/robots/robotiq_gripper/robotiq_arg85_description.URDF', 'robot-assets/urdfs/robots/anymal/anymal.urdf', 'robot-assets/urdfs/robots/franka_panda/panda.urdf', 'robot-assets/urdfs/robots/ginger_robot/gingerurdf.urdf', 'robot-assets/urdfs/robots/halodi/eve_r3.urdf', 'robot-assets/urdfs/robots/kinova/kinova.urdf', 'robot-assets/urdfs/robots/kuka_iiwa/model.urdf', 'robot-assets/urdfs/robots/pr2/pr2.urdf', 'robot-assets/urdfs/robots/ur10/ur10_robot.urdf', 'robot-assets/urdfs/robots/ur5/ur5_gripper.urdf', 'robot-assets/urdfs/robots/yumi/yumi.urdf']

import urdfpy
import urdf_parser_py
import yourdfpy

from functools import partial

def load_urdfs(fnames, load_fn):
    results = {fname: None for fname in fnames}
    for fname in fnames:
        try:
            x = load_fn(fname)
            results[fname] = x
        except:
            pass
    print(sum([1 for x, y in results.items() if y is not None]), "/", len(fnames))
    return results

# parsing success rate
load_urdfs(robot_assets, urdfpy.URDF.load)
load_urdfs(robot_assets, urdf_parser_py.urdf.URDF.load)
load_urdfs(robot_assets, yourdfpy.URDF.load)

# parsing times
%timeit load_urdfs(robot_assets, urdfpy.URDF.load)
%timeit load_urdfs(robot_assets, urdf_parser_py.urdf.URDF.load)
%timeit load_urdfs(robot_assets, yourdfpy.URDF.load)
%timeit load_urdfs(robot_assets, partial(yourdfpy.URDF.load, load_meshes=False, generate_scene_graph=False))

# fairer comparison with yourdfpy
urdfpy_fnames = [x for x, y in load_urdfs(robot_assets, urdfpy.URDF.load).items() if y is not None]
%timeit load_urdfs(urdfpy_fnames, yourdfpy.URDF.load)

# fairer comparison with urdf_parser_py
urdfparser_fnames = [x for x, y in load_urdfs(robot_assets, urdf_parser_py.urdf.URDF.from_xml_file).items() if y is not None]
%timeit load_urdfs(urdfparser_fnames, functools.partial(yourdfpy.URDF.load, load_meshes=False, generate_scene_graph=False))
```

<!--
# Visualization


cam_rot = s.camera_transform
robot_assets = glob.glob('/data/robot-assets/urdfs/robots/**/*.urdf')
for i, fname in enumerate(robot_assets):
  try:
    s = yourdfpy.URDF.load(fname).scene
    cam_T = s.camera.look_at(points=s.convex_hull.vertices, rotation=cam_rot) # distance=2.6
    s.camera_transform = cam_T
    png = s.save_image()
    with open(f"/tmp/test{i:02}.png", 'wb') as f:
        f.write(png)
  except Exception as e:
        print(e)

~/crop_image_horizontal.sh /tmp/test*png
montage /tmp/test*png -geometry +50+0 -tile x1 /tmp/montage_geom.jpg
-->

<!--
How to deploy

git tag -l
rm dist/*
rm -rf build/

# https://pyscaffold.org/en/latest/faq.html#version-faq
git gui # commit something?
git tag v<semver>
git push origin main
python setup.py bdist_wheel
twine upload -r testpypi dist/*

python -m pip install --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple yourdfpy==v<semver>

twine upload dist/*
-->
