# r2p2

Simple robot simulator written in Python 3.
Module based, and focused on extensibility.
Check requirements.txt for more detailed information on what Python modules need to be installed.

Developed and tested on Python 3.7.

## Setup

In order to run the simulation, you first need to install its dependencies. For that, follow these steps. All dependencies should be available using pip:
1. Install numpy:
```Bash
pip install numpy
```
Depending on how your environment is configured, your operating system might be unable to identify pip as a Python module. In that case, try running the following:
```Bash
python -m pip install numpy
```
2. Install matplotlib:
```Bash
pip install matplotlib
```
Once again, if that doesn't work, the following might solve the problem:
```Bash
python -m pip install matplotlib
```
3. Install Pillow:
```Bash
pip install Pillow
```
or
```Bash
python -m pip install pillow
```
per usual.

4. Install scikit-learn:
```Bash
pip install scikit-learn
```
or
```Bash
python -m pip install scikit-learn
```
5. Install scipy:
```Bash
pip install scipy
```
or
```Bash
python -m pip install scipy
```

### Conda environment
Creating a Conda environment to run the simulator is extremely easy. Simply start your Conda console (if you are on Windows), or a regular command prompt and execute:
```Bash
conda create --name [name of the new environment] python=3.7
```
replacing `[name of the new environment]` for the actual name you want to use, for instance, `r2p2`, and pressing `y` when prompted to.

Next, enter the environment using
```Bash
conda activate [name of the new environment]
```
and confirm that the result of 
```Bash
python --version
```
is `Python 3.7.x`.

Afterwards, install the dependencies as described above. You may also use 
```Bash
conda install [package name]
```
instead of pip.

## Running the simulator

In order to run the simulator, simply execute the following command:
```Bash
python r2p2.py --scenario [path to scenario configuration JSON]
```

Additionally, if you would like to see the program's help, you may execute
```Bash
python r2p2.py --help
```
or
```Bash
python r2p2.py -h
```

Finally, if you would like to check the current version, use
```Bash
python r2p2.py --version
```

## Configuration
The simulator uses three main types of configuration files:
 
 - scenario-XYZ.json:  configures specific scenarios for the simulator to run. It must contain the following:
 
	 -  stage: path to the image defining the map that will be used.
	 - robot: path to the robot's configuration file.
	 - controller: path to the controller's configuration file.
	 - gui: boolean indicating whether to use a graphical user interface. Set to true by default.
 - robot.json:  configures robots. It must contain information related to its radius and sonars, as well as parameters such as its maximum linear velocity and starting position. Please, refer to the sample files provided with the simulator to get more information.
 - controller-XYZ.json:  configures specific controllers. Required fields are completely controller-dependant. Please, refer to the sample configuration files provided with this simulator, as well as the controller in question's documentation in order to properly generate controller configuration files.

If you are using this simulator for path planning purposes, please, do not replace the PID constants in order to avoid more erratic behavior cropping up. If in doubt, the default constants are:
```Json
  "ap": 2
  "ai": 0.015
  "ad": 0.016
  "lp": 0.5
  "li": 0.015
  "ld": 0.016
 ```
 ### About grid sizes for path-planning
 In order to modify the size of a grid, edit the controller's `grid_size` attribute (defaulted to 30 in the example provided). It's advisable to use values between 15 and 40 in order not to lose precision without compromising efficiency.
 
 `grid_size` should always have a smaller value than the smallest dimension of the map in use. This is, a grid_size of 500 on a map of 450 by 425 is likely to cause faulty behavior, due to the dimensions used.

## Creating stage files
In order to create a stage file, simply create an image file using your image editor of choice. Only black is taken as a wall by the simulator, although darker areas will potentially generate areas considered harder to traverse.

Any color may be used in order to fill in darker areas, although the simulator will automatically transform it into levels on run time.
