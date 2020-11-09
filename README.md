# Assembly Information Model


## Requirements

* Minimum OS: **Windows 10 Pro** or **Mac OS Sierra 10.12**
* [Rhinoceros 3D 6.0](https://www.rhino3d.com/)
* [Anaconda 3](https://www.anaconda.com/products/individual)
* Git: [official command-line client](https://git-scm.com/) and visual GUI (e.g. [Github Desktop](https://desktop.github.com/))
* [VS Code](https://code.visualstudio.com/) with the following `Extensions`:
  * `Python` (official extension)


## Installation

### Compas Installation 
(via your Anaconda Terminal)
    
    (base)  conda config --add channels conda-forge
    (base)  conda create -n afab20 python=3.8 compas_fab=0.13 --yes
    (base)  conda activate afab20
    (afab20) python -m compas_rhino.install -v 6.0 -p compas compas_ghpython compas_rhino
    
### Verify Installation

    (afab20) pip show compas_fab
    Name: compas-fab
    Version: 0.13.1
    Summary: Robotic fabrication package for the COMPAS Framework



Make sure you setup your local development environment correctly:

* Clone the `assembly_information_model <https://github.com/augmentedfabricationlab/assembly_information_model>`_ repository.
* Install development dependencies and make the project accessible from Rhino (change to repository directory in the Anaconda prompt):

::

    pip install -r requirements-dev.txt
    invoke add-to-rhino
    pip install your_filepath_to_assembly_information_model 


### Installation in editable mode


::

    pip install -e your_filepath_to_assembly_information_model 



Credits
-------------

This package was created by `Kathrin Doerfler <doerfler@tum.de>`_ `@kathrindoerfler <https://github.com/kathrindoerfler>`_ at `@augmentedfabricationlab <https://github.com/augmentedfabricationlab>`_. This package is based on `compas_assembly <https://github.com/BlockResearchGroup/compas_assembly>`_ by `@BlockResearchGroup <https://github.com/BlockResearchGroup>`_


