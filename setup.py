import os
from setuptools import setup, find_packages


def read(fname):
    with open(os.path.join(os.path.dirname(__file__), fname)) as f:
        return f.read()


setup(
    name="nclt2rosbag",
    version="1.0",
    author="Bierschneider Christian",
    author_email="christian.bierschneider@web.de",
    description="downloads, extracts, converts and visualizes the nclt dataset",
    long_description=read('README.md'),
    url='',
    license=read('LICENSE'),
    python_requires='>=2.7',
    py_modules=["nclt2rosbag", "definitions"],
    scripts=['nclt2rosbag.py'],
    #packages=["src", "src.converter", "src.downloader", "src.extractor", "src.transformer", "src.visualizer"],
    #packages=find_packages(exclude=('plots','raw_data', 'rosbags')),
    packages=find_packages(),
    #package_data={'': ['requirements.txt']},
    #package_data={'cfg': ['configuration.json'], 'rviz': ['config.rviz'], '': ['requirements.txt']},
    data_files=[('cfg', ['cfg/configuration.json'])],
    install_requires=["numpy", "simplekml", "matplotlib"],
    #cmdclass={
    #    'install': PostInstallCommand
    #},
)
