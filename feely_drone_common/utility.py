import importlib.resources

def get_urdf_path(name: str) -> str:
    with importlib.resources.path("feely_drone_common.assets", name) as path:
        return str(path)
    