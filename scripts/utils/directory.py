import os

def get_pkg_dir():
    return os.path.abspath(
        os.path.join(
            os.path.realpath(__file__),
            "../../.."
        )
    )


def get_model_dir():
    pkg_dir = get_pkg_dir()
    return os.path.join(pkg_dir, "models")
