from setuptools import setup, find_packages

# read the contents of your README file
from os import path
this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'README.md'), encoding='utf-8') as f:
    lines = f.readlines()

# remove images from README
lines = [x for x in lines if '.png' not in x]
long_description = ''.join(lines)

setup(
    name="deftenv",
    packages=[
        package for package in find_packages() if package.startswith("deftenv")
    ],
    install_requires=[
        "numpy>=1.13.3",
        "h5py",
        "imageio",
        "imageio-ffmpeg",
        "opencv",
        "pybullet"
    ],
    eager_resources=['*'],
    include_package_data=True,
    python_requires='>=3',
    description="Skill-based manipulation tasks",
    author="Danfei Xu",
    url="https://github.com/danfeiX/deftenv",
    author_email="danfei@cs.stanford.edu",
    version="0.0.1",
    long_description=long_description,
    long_description_content_type='text/markdown'
)
