from setuptools import setup, find_packages
import sys, os

version = '0.34'

this_directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(this_directory, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()
    
setup(name='camproject',
      version=version,
      description="a camera projection lib to calculate the ray direction of a pixel and vice versa",
      long_description=long_description,
      long_description_content_type='text/markdown',
      classifiers=[
       'Development Status :: 4 - Beta',
          'Intended Audience :: Developers',
          'License :: OSI Approved :: MIT License',
          'Programming Language :: Python :: 3'
      ], # Get strings from http://pypi.python.org/pypi?%3Aaction=list_classifiers
      keywords='camera pin hole projection brown lut',
      author='Martin Israel',
      author_email='martin.israel@dlr.de',
      url='',
      license='MIT license',
      packages=find_packages(exclude=['ez_setup', 'examples', 'tests']),
      include_package_data=True,
      zip_safe=False,
      install_requires=[
        'numpy',
          # -*- Extra requirements: -*-
      ],
      entry_points="""
      # -*- Entry points: -*-
      """,
      )
