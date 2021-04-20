# CODING GUIDES

## 1. Creating a new package

`ros2 pkg create --build-type ament_python name_of_pkg`

### Thing to watch out for:
* Add `<exec_depend>lib_name</exec_depend>` in the package.xml file
* Add under console script in setup.py in this format: 
`'name_of_command = pkg_name.python_program_name:main',`

### To build the packages:
* `cd ~/colcon_ws`
* `source install/setup.bash`
* `colcon build --packages-select pkg_name`

### To run the package:
* `ros2 run pkg_name name_of_command`