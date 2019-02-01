import os
try:
    from setuptools import setup
    from setuptools.command.install import install
except ImportError:
    from distutils.core import setup
    from distutils.command.install import install


def read(fname):
    with open(os.path.join(os.path.dirname(__file__), fname)) as f:
        return f.read()


setup(
    name="nclt2rosbag",
    version="1.0",
    author="Bierschneider Christian",
    author_email="christian.bierschneider@web.de",
    description="downloads, extracts, converts, visualizes the nclt dataset",
    long_description=read('README.md'),
    license='MIT',
    py_modules=["nclt2rosbag", "definitions"],
    #scripts=['', ''],
    packages=["src"],
    #package_data={},
    install_requires=["numpy", "simplekml", "matplotlib"],
    #cmdclass={
    #    'install': PostInstallCommand
    #},
)