from setuptools import find_packages, setup

package_name = "dynoturtle_simulation"

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
            "turtle_spawner = dynoturtle_simulation.turtle_spawner:main",
            "simulator_extensions = dynoturtle_simulation.simulator_extensions:main",
        ],
    },
)
