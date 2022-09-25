#!/usr/bin/python3

from setuptools import setup, find_packages, Extension

try:
	from Cython.Build import cythonize
except ImportError:
	cython_modules = [Extension('madcad.core', ['madcad/core.c'])]
else:
	cython_modules = cythonize(['madcad/core.pyx'])	#, annotate=True)

setup(
	# package declaration
	name='pymadcad',
	version='0.13.2',
	python_requires='>=3.8',
	install_requires=[
		'pyglm>=2.5.5',
		'moderngl>=5.6',
		'numpy>=1.1',
		'scipy>=1.3',
		'PyQt5>=5',
		'Pillow>=5.4',
		'pyyaml>=5',
		'arrex>=0.5',
		],
	extras_require={
		'PLY': ['plyfile>=0.7'],
		'STL': ['numpy-stl>=2'],
		'OBJ': ['PyWavefront>=1.3'],
		},
	# source declaration
	packages=find_packages(),
	ext_modules=cython_modules,
	package_data={
		'madcad': [
			'shaders/*.frag', 
			'shaders/*.vert', 
			'textures/*.png', 
			'*.py', 
			'*.pyx', 
			'*.c', 
			],
		'': ['COPYING', 'COPYING.LESSER', 'README.md', 'requirements.txt'],
		},
	
	# metadata for pypi
	author='Yves Dejonghe',
	author_email='jimy.byerley@gmail.com',
	description="Simple yet powerful CAD (Computer Aided Design) library, written with Python",
	long_description=open('README.md').read(),
	long_description_content_type='text/markdown',
	license='GNU LGPL v3',
	url='https://github.com/jimy-byerley/pymadcad',
	keywords='CAD 3D parametric mesh kinematic solid solver part design',
	classifiers=[
		'License :: OSI Approved :: GNU General Public License v3 or later (GPLv3+)',
		'Topic :: Scientific/Engineering',
		'Development Status :: 3 - Alpha',
		'Programming Language :: Python :: 3.8',
		'Programming Language :: Python :: 3.9',
		'Programming Language :: Python :: 3.10',
		'Programming Language :: Python :: Implementation :: CPython',
		'License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)',
		'Intended Audience :: Science/Research',
		'Intended Audience :: Manufacturing',
		'Intended Audience :: Education',
		'Topic :: Scientific/Engineering',
		'Topic :: Scientific/Engineering :: Visualization',
		'Environment :: X11 Applications :: Qt',
		],
	)
