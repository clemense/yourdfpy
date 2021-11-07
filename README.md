# yourdfpy

Yet anOther URDF parser for Python. Yup, it's another one. Deal with it.

Yourdfpy is a simpler and easier-to-use library for loading, manipulating, validating, saving, and visualizing URDF files.

## Installation

You can install yourdfpy directly from pip:
```
pip install yourdfpy
```

<!--
## But why?!?

Insert RANT here
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

<!--
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