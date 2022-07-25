import yourdfpy as urdf

def parse_panda():
    urdf_file = "panda.urdf"
    __import__('pdb').set_trace()
    my_urdf = urdf.URDF.load(urdf_file)


if __name__ == "__main__":
    parse_panda()
