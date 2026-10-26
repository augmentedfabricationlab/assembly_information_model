# Assembly Information Model (Compas 2 version)

This repository provides data structures, tools and methods for assembly information modeling.

This README describes the setup for the **`cellularized_assembly`** branch.

## Requirements

* Rhino 8 / Grasshopper
* [Anaconda Python](https://www.anaconda.com/download)
* [Github Desktop](https://desktop.github.com/)
* [Visual Studio Code](https://code.visualstudio.com/)

## Installation

### Cloning the repository to your workstation

1. Create a **workspace** folder in File Explorer:

    `C:\Users\your_username\workspace`

2. Open **GitHub Desktop** and click **Current Repository** (top left), then **Add > Clone Repository...**.

3. Select the **URL** tab and paste:

    `https://github.com/augmentedfabricationlab/assembly_information_model.git`

4. Set the **Local Path** (bottom) so it looks like:

    `C:\Users\your_username\workspace\assembly_information_model`

5. Click **Clone**.

6. Switch to the **cellularized_assembly** branch: click **Current Branch** (top, next to Current Repository) and select `cellularized_assembly`.

### Setting up your GitHub Copilot account

1. Create a free [GitHub account](https://github.com/signup) if you do not have one yet.

2. As a TUM student, you are eligible for free **GitHub Pro** and **GitHub Copilot Pro**. Request them by following the [TUM GitHub guide](https://collab.dvb.bayern/spaces/TUMsoftware/pages/2662964764/GitHub) (TUM login required).

### Setting up VS Code and extensions

1. Install the extensions. Open the Extensions panel (`Ctrl+Shift+X`), search for each one and click **Install**:
    * **Python** (by Microsoft): syntax highlighting and code help for Python files.
    * **GitHub Copilot** (by GitHub): the AI assistant. This also installs **GitHub Copilot Chat**.

2. Log in to your Copilot account:
    * Click the Copilot icon in the VS Code status bar (bottom right) or the account icon (bottom left) and choose **Sign in with GitHub**.
    * Sign in with your GitHub account in the browser and authorize VS Code.

3. Check that it works: open the Copilot Chat panel (`Ctrl+Alt+I`) and send a short message. If you get an answer, you are ready.

### Installing COMPAS FAB

In an Anaconda prompt, create and activate an environment:

    (base)  conda create -n your_env_name -c conda-forge compas_fab
    (base)  conda activate your_env_name

Verify the installation:

    (your_env_name) python -m compas_fab
    Yay! COMPAS FAB is installed correctly!

Install COMPAS FAB into Rhino's Python from PyPI. First find the Rhino 8 Python executable:

    (your_env_name) python -m compas_rhino.print_python_path

Its path should look something like this:

    C:\Users\your_username\.rhinocode\py39-rh8\python.exe

Then install with that executable:

    (your_env_name) C:\Users\your_username\.rhinocode\py39-rh8\python.exe -m pip install compas_fab

### Making the AIM repository accessible in Rhino 8

This tells Rhino's Python where to find the AIM code, so you can `import` it in Grasshopper.

1. Open **Rhino 8** and start **Grasshopper** (type `Grasshopper` in the Rhino command line).

2. Add a **Python 3 Script** component to the canvas. The first time, wait for it to finish loading.

3. Double-click the component to open the script editor.

4. In the editor's top bar, go to **Tools > Options**.

5. Open the **Python 3** tab.

6. Under **Module search paths**, click **+** and add the path to the `src` folder of your cloned repository:

    `C:\Users\your_username\workspace\assembly_information_model\src`

7. Click **Save**.

8. Check that it works: in the component, run `import assembly_information_model`. If there is no error, the path is set up correctly.

## Credits

This package was created by [Kathrin Doerfler](doerfler@tum.de) [@kathrindoerfler](https://github.com/kathrindoerfler) at [@augmentedfabricationlab](https://github.com/augmentedfabricationlab). This package is based on [compas_assembly](https://github.com/BlockResearchGroup/compas_assembly) by [@BlockResearchGroup](https://github.com/BlockResearchGroup)
