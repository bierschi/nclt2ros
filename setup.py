import os
from setuptools import setup, find_packages


def read(fname):
    with open(os.path.join(os.path.dirname(__file__), fname)) as f:
        return f.read()


EXCLUDE_FROM_PACKAGES = ['nclt2rosbag.raw_data',
                         'nclt2rosbag.rosbags',
                         'nclt2rosbag.plots']


setup(
    name="nclt2rosbag",
    version="1.0",
    author="Bierschneider Christian",
    author_email="christian.bierschneider@web.de",
    description="downloads, extracts, converts and visualizes the NCLT dataset",
    long_description=read('README.md'),
    url='',
    license=read('LICENSE'),
    python_requires='>=2.7',
    py_modules=["nclt2rosbag"],
    #entry_points={'console_scripts': ['nclt2rosbag = nclt2rosbag.nclt2rosbag:main', ]},
    packages=find_packages(exclude=EXCLUDE_FROM_PACKAGES),
    package_dir={'nclt2rosbag': 'nclt2rosbag'},
    package_data={'nclt2rosbag': ['cfg/configuration.json', 'rviz/config.rviz']},
    data_files=[('tf_tree', ['tf_tree/frames.pdf'])],
    install_requires=["numpy", "simplekml", "matplotlib"],
)
