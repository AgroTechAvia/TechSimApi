## techsimapi


### This repository contains the API for the AgroTechAvia simulators. This repository also contains several examples of how to interact with the API.

### [1] Setup environment

After installation, you need to create a virtual environment. The environment can be created in any directory on your computer, but we recommend creating it in the folder where this module is stored.

```bash
python -m venv .venv # Windows

python3 -m venv .venv # Linux
```
To activate the environment:

```bash
.\.venv\Scripts\activate #Windows

source .venv/bin/activate # Linux
```
To fully configure all dependencies, run the file setup_all.py

```bash
python setup_all.py
```
This completes the installation!

### [2] Examples

The examples folder contains examples of the program.

This version of the API supports:
1. Receiving images from cameras
2. Receiving stereo images (depth camera)
3. Receiving thermal images (thermal camera)
4. 360deg lidar
5. Sonars
6. Rangefinders
7. LED backlight control
8. Receiving kinematic data (for debugging)
9. Drone control (see inavmspapi)



