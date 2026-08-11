from distutils.core import setup

package_name = "ekf_local_enu"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_dir={"": "src"},
)
