from setuptools import setup, find_packages

setup(
    name="magpie_control",  # Replace with your project name
    version="0.1.0",
    description="magpie control code for ur5, gripper, and camera",
    author="",
    author_email="",
    url="https://github.com/correlllab/magpie_control",  # Replace with the URL of your GitHub repo
    packages=find_packages(),  # Automatically find and include all packages
    install_requires=[  # List your project's dependencies
        # e.g., "numpy>=1.18.0",
    ],
    python_requires='>=3.9',  # Specify the Python version required
)

