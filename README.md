# Assembly Information Model (Compas 2 version)

This repository provides data structures, tools and methods for assembly information modeling.

## Requirements

* Windows 10 Professional
* Rhino 8 / Grasshopper
* [Anaconda Python](https://www.anaconda.com/download)
* [Visual Studio Code](https://code.visualstudio.com/)
* [Github Desktop](https://desktop.github.com/)
* potentially: [Microsoft Visual C++](https://www.scivision.dev/python-windows-visual-c-14-required/)

## Installation

#### Installation COMPAS FAB
    
    (base)  conda create -n your_env_name -c conda-forge compas_fab
    (base)  conda activate your_env_name

#### Verify Installation of COMPAS FAB

    (your_env_name) python -m compas_fab
    Yay! COMPAS FAB is installed correctly!   

#### Installation of COMPAS FAB on Rhino from PyPI

    (your_env_name) python -m compas_rhino.print_python_path
    (your_env_name) C:\Users\your_user_name\.rhinocode\py39-rh8\python.exe -m pip install compas_fab


Make sure you set up your local development environment correctly:

* Clone the [assembly_information_model](https://github.com/augmentedfabricationlab/assembly_information_model) repository.
* Change to the branch 'compas2'!!!!!!!!
* Install development dependencies and make the project accessible from Rhino (change to repository directory in the Anaconda prompt):

#### Making the AIM repository accessible in Rhino 8

Find the Rhino 8 Python executable by running the following in a terminal or command prompt:

    (your_env_name) python -m compas_rhino.print_python_path

Your Rhino 8 Python path should look something like this:

    C:\Users\your_user_name\.rhinocode\py39-rh8\python.exe
    
Then you can pip install all dependencies using the file path of the Rhino 8 Python executable:

    (your_env_name) your_py39-rh8_path -m pip install your_filepath_to_assembly_information_model


In editable mode:

    (your_env_name) your_py39-rh8_path pip install -e your_filepath_to_assembly_information_model


## Credits

This package was created by [Kathrin Doerfler](doerfler@tum.de) [@kathrindoerfler](https://github.com/kathrindoerfler) at [@augmentedfabricationlab](https://github.com/augmentedfabricationlab). This package is based on [compas_assembly](https://github.com/BlockResearchGroup/compas_assembly) by [@BlockResearchGroup](https://github.com/BlockResearchGroup)


