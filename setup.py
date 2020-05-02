from setuptools import setup, find_packages
from Cython.Build import cythonize

setup(
	# package declaration
	name='pymadcad',
	version='0.1',
	install_requires=['pyglm>=1.2', 'moderngl>=5.6', 'numpy>=1.1', 'PIL>=5.4', 'PyQt5>=5'],
	extras_require={
		'PLY': ['plyfile>=0.7'],
		'STL': ['numpy-stl>=2'],
		'OBJ': ['PyWavefront>=1.3'],
		},
	# source declaration
	packages=find_packages(),
	ext_modules=cythonize(['madcad/core.pyx'], annotate=True),
	
	# metadata for pypi
	author='jimy byerley',
	author_email='jimy.byerley@gmail.com',
	description="core library of the madcad project, it's time to throw parametric softwares out !",
	)
