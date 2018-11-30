from setuptools import setup, find_packages
import sys, os

version = '0.31'

setup(name='camera',
      version=version,
      description="a camera projection lib to calculate the ray direction of a pixel and vice versa",
      long_description="""\
""",
      classifiers=[], # Get strings from http://pypi.python.org/pypi?%3Aaction=list_classifiers
      keywords='camera pin hole projection brown lut',
      author='Martin Israel',
      author_email='martin.israel@dlr.de',
      url='',
      license='MIT license',
      packages=find_packages(exclude=['ez_setup', 'examples', 'tests']),
      include_package_data=True,
      zip_safe=False,
      install_requires=[
          # -*- Extra requirements: -*-
      ],
      entry_points="""
      # -*- Entry points: -*-
      """,
      )
