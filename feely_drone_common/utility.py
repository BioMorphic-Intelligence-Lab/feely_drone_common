import importlib.resources

def get_urdf_path(name: str) -> str:
    if "cylinder" in name:
        with importlib.resources.path("feely_drone_common.assets.cylinders", name) as path:
            return str(path)
    else:
        with importlib.resources.path("feely_drone_common.assets", name) as path:
            return str(path)
    