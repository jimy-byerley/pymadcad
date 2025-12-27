# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder.
#
# This file does only contain a selection of the most common options. For a
# full list see the documentation:
# http://www.sphinx-doc.org/en/master/config

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#

# -- Project information -----------------------------------------------------

project = 'pymadcad'
copyright = '2019-2021, jimy byerley'
author = 'jimy byerley'

version = '0.17'			# The short X.Y version
release = 'v'+version	# The full version, including alpha/beta/rc tags


# -- General configuration ---------------------------------------------------
needs_sphinx = '5.1'
# sphinx extensions
extensions = [
    'sphinx.ext.autodoc',  # documentation for docstrings
    'sphinx.ext.intersphinx',  # for external links
    'sphinx.ext.viewcode',  # for links to source code
    'sphinx.ext.napoleon',   # docstring format parser
    'sphinx.ext.autosectionlabel',  # generate links to functions and sections
    'sphinx_collapse',        # allow to collapse portions of text
	'sphinxcontrib.mermaid',  # support for simple markdown schematics
    'myst_parser',            # support for markdown
    'sphinx_rtd_theme',       # page theme
	'sphinx_subfigure',
]
myst_enable_extensions = [
	'dollarmath',
	'smartquotes',
	'tasklist',
	]
myst_heading_anchors = 3

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']
source_suffix = ['.rst', '.md']
master_doc = 'index' # The master toctree document

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# The name of the Pygments (syntax highlighting) style to use.
#pygments_style = 'github-dark'  # alternative to the highlight defined in custom.css
add_module_names = False	# remove module names from function docs
default_role = 'code'
primary_domain = 'py'


# -- Options for HTML output -------------------------------------------------
html_logo = 'logo.png'
html_favicon = "logo.ico"
html_static_path = ['static']	# path to custom static files, such as images and stylesheets

html_theme = 'sphinx_rtd_theme'

napoleon_numpy_docstring = False
napoleon_google_docstring = True

# -- Options for HTMLHelp output ---------------------------------------------

# Output file base name for HTML help builder.
htmlhelp_basename = 'pymadcaddoc'

# -- Options for Epub output -------------------------------------------------

# Bibliographic Dublin Core info.
epub_title = project
epub_exclude_files = ['search.html']



def setup(app):
    app.add_css_file('custom.css')
