from setuptools import setup, find_packages

setup(
    name="ros2-interfaces-python",
    version="0.1.0",
    packages=find_packages(where="ros2_interfaces_python"),
    package_dir={"": "ros2_interfaces_python"},
    install_requires=[
        # No external dependencies - pure Python dataclasses
    ],
    extras_require={
        "dev": [
            "pytest>=6.0",
            "pytest-cov>=2.0",
            "black>=21.0",
            "isort>=5.0",
            "flake8>=3.0",
        ],
    },
    author="Your Name",
    author_email="your.email@example.com",
    description="Simplified Python dataclasses for ROS 2 interface types",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/your_org/ros2-interfaces-python",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: System :: Networking",
    ],
    python_requires=">=3.8",
)
