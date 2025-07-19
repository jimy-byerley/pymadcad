import os
import shutil
from pathlib import Path

from Cython.Build import cythonize
from setuptools import Distribution
from setuptools import Extension
from setuptools.command.build_ext import build_ext

distribution = Distribution({
    "name": "package",
    "ext_modules": cythonize(['madcad/core.pyx'])
})

cmd = build_ext(distribution)
cmd.ensure_finalized()
cmd.run()

# Copy built extensions back to the project
for output in cmd.get_outputs():
    output = Path(output)
    relative_extension = output.relative_to(cmd.build_lib)

    shutil.copyfile(output, relative_extension)
    mode = os.stat(relative_extension).st_mode
    mode |= (mode & 0o444) >> 2
    os.chmod(relative_extension, mode)
