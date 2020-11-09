# Assembly Information Model


## Requirements

* COMPAS

## Installation

### Compas Installation 
(via your Anaconda Terminal)
    
    (base) conda config --add channels conda-forge

#### Windows
    (base) conda create -n your_env_name  python=3.8 compas_fab=0.13 --yes
    (base) conda activate your_env_name 

#### Mac
    (base) conda create -n afab20 python=3.8 compas_fab=0.13 python.app --yes
    (base) conda activate your_env_name 
    

### Verify Installation

    (your_env_name) pip show compas_fab
    Name: compas-fab
    Version: 0.13.1
    Summary: Robotic fabrication package for the COMPAS Framework
    ...

### Install on Rhino

    (your_env_name) python -m compas_rhino.install

NOTE: This installs to Rhino 6.0, use `-v 5.0` if needed.



Make sure you setup your local development environment correctly:

* Clone the `assembly_information_model <https://github.com/augmentedfabricationlab/assembly_information_model>`_ repository.
* Install development dependencies and make the project accessible from Rhino (change to repository directory in the Anaconda prompt):

    pip install -r requirements-dev.txt
    invoke add-to-rhino
    pip install your_filepath_to_assembly_information_model 


### Installation in editable mode


    (your_env_name) pip install -e your_filepath_to_assembly_information_model 




## Credits

This package was created by `Kathrin Doerfler <doerfler@tum.de>`_ `@kathrindoerfler <https://github.com/kathrindoerfler>`_ at `@augmentedfabricationlab <https://github.com/augmentedfabricationlab>`_. This package is based on `compas_assembly <https://github.com/BlockResearchGroup/compas_assembly>`_ by `@BlockResearchGroup <https://github.com/BlockResearchGroup>`_


