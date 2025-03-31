from setuptools import setup, find_packages

setup(
    name="feely_drone_common",  # Package name
    version="0.1.0",    # Version number
    packages=find_packages(),  # Automatically find packages
    install_requires=[    # Dependencies
        "numpy",         
        "matplotlib"
    ],
    package_data={
        "feely_drone_common": ["assets/*.urdf"],  # Include these files
    },
    author="Anton Bredenbeck",
    description="Common Implementations used for the Feely Drone project",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    #classifiers=[
    #    "Programming Language :: Python :: 3",
    #    "License :: OSI Approved :: MIT License",
    #],
    #python_requires=">=3.7",  # Minimum Python version
)
