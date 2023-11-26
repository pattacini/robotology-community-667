robotology-community-667
========================

[![Gitpod](https://gitpod.io/button/open-in-gitpod.svg)](https://gitpod.io/from-referrer)

Robotology community support files.

This tiny project allows for reading the iCub arm joints' values from a text file (ordered by rows) to then compute the forward kinematics using `iCub::iKin` and get the end-effector pose.
- The input text file contains the joints given in degrees and organized by rows.
- Columns can be 10 or 7 depending on whether the torso is used or not.
- The end-effector pose contains 7 numbers comprising the position (in meters) and the orientation expressed in axis-angle notation.

### Build the project
The required dependencies include [specific releases](https://github.com/pattacini/robotology-community-667/blob/master/CMakeLists.txt#L8-L10) of `yarp`, `icub-main`, and `icub-contrib-common`.
You can retrieve dependencies following the instructions on our main [documentation](https://icub-tech-iit.github.io/documentation/sw_versioning_table/) page.

Alternatively, you can rely on a [docker image](https://github.com/vvv-school/vvv-school.github.io/pkgs/container/gitpod).
You may also consider resorting to [Gitpod](https://www.gitpod.io/).

```console
cmake -S . -B build
cmake --build build/ --target install
```

### Run the executable
To get the online help, run:
```console
robcom-667 --help
``` 
