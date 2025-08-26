from setuptools import find_packages, setup

package_name = "dynoturtle_behaviors"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rotate_action = dynoturtle_behaviors.actions.rotate_action:main",
            "move_action = dynoturtle_behaviors.actions.move_action:main",
            "simple_tree = dynoturtle_behaviors.trees.simple_tree:main",
        ],
    },
)
