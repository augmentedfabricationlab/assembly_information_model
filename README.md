# Assembly Information Model


## Requirements

* COMPAS

## Installation

### Compas Installation 
(via your Anaconda Terminal)
    
    (base) conda config --add channels conda-forge
    (base) conda create -n your_env_name COMPAS
 


### Verify Installation

    (base) conda activate your_env_name
    (your_env_name) python -m compas
#
    Yay! COMPAS is installed correctly!

    COMPAS: 0.17.2
    Python: 3.8.6 | packaged by conda-forge | (default, Oct  7 2020, 18:22:52) [MSC v.1916 64 bit (AMD64)]   

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


