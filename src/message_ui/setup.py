from setuptools import setup

package_name = "message_ui"

setup(
    name=package_name,
    version="1.0.5",
    packages=[package_name],
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/resource", ["resource/MessageUI.ui"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["plugin.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="See package.xml",
    maintainer="See package.xml",
    maintainer_email="See package.xml",
    keywords=["ROS"],
    description=("See package.xml"),
    license="See package.xml",
    entry_points={
        "console_scripts": [
            "message_ui = " + package_name + ".main:main",
        ],
    },
)
