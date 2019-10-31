# -- Project information -----------------------------------------------------

project = 'cepton_sdk'
copyright = '2018, Cepton Technologies'
author = 'Cepton Technologies'

# -- General configuration ------------------------------------------------

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.mathjax',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode'
]
templates_path = ['_templates']
source_suffix = '.rst'
master_doc = 'index'
language = None
exclude_patterns = []
pygments_style = 'sphinx'

# -- Options for HTML output ----------------------------------------------

html_theme = 'classic'
html_static_path = []
html_sidebars = {
    '**': [
        'relations.html',  # needs 'show_related': True theme option to display
        'searchbox.html',
    ]
}

# -- Options for LaTeX output ---------------------------------------------

latex_elements = {}
latex_documents = [
    (master_doc, 'cepton_sdk.tex', 'cepton\\_sdk Documentation',
     'Cepton Technologies', 'manual'),
]
