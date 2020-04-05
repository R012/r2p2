# r2p2 robot simulator

Simple robot simulator written in Python 3.
Module based, and focused on extensibility.
Check requirements.txt for more detailed information on what Python modules need to be installed.

Developed and tested on Python 3.7.

## Download

R2p2 is hosted in GitHub, so you can clone the repository as usual in Git:

```Bash
git clone https://github.com/ISG-UAH/r2p2.git
```
As an alternative you can download a zip file with the simulator as a direct download (checkout out the green button in the [r2p2 repository](https://github.com/ISG-UAH/r2p2)).

## Conda environment

We highly recommend to install r2p2 and its dependencies in a Conda environment as it maintains all the system independent from your host.

Install python 3.X version of [Anaconda](https://www.anaconda.com/distribution/) and then create a new conda environment.  Creating a Conda environment to run the simulator is extremely easy. Simply start your Conda console (if you are on Windows), or a regular command prompt and execute:

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
is `Python 3.7.x`. Please observe that the prompt displays the environment you are running.

Afterwards, install the dependencies using pip and the requirements.txt file.

```Bash
 pip3 install -r requirements.txt
```
It should download and install automatically all the r2p2 dependencies.

And that's it, you are ready to run r2p2.


## Running the simulator

In order to run the simulator, simply execute the following command from the r2p2 folder:

```Bash
python r2p2.py
```

which runs a default scenario with a teleoperated robot on it. You can command 
the robot by using the arrows keys in the keyboard or a joystick.

Changing the simulated scenario is straitfoorward, just use the scenario parameter.

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
 - controller-XYZ.json:  configures specific controllers. The only required field is "class", which specifies the class that implements the controller. Each controller type require different fields, please, refer to the sample configuration files provided with this simulator, as well as the controller in question's documentation in order to properly generate controller configuration files.

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
 In order to modify the size of a grid, edit the controller's `grid_size` attribute (defaulted to 30 in the example provided) in the `controller-pathplanning.json` file. It's advisable to use values between 15 and 40 in order not to lose precision without compromising efficiency.
 
 `grid_size` should always have a smaller value than the smallest dimension of the map in use. This is, a grid_size of 500 on a map of 450 by 425 is likely to cause faulty behavior, due to the dimensions used.

## Creating scenario files
In order to create a scenario file, simply create an image file using your image editor of choice. Only black is taken as a wall by the simulator, although darker areas will potentially generate areas considered harder to traverse.

Any color may be used in order to fill in darker areas, although the simulator will automatically transform it into levels on run time.
