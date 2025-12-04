# hexapod_kinematics
![build](https://github.com/conroy-cheers/hexapod_kinematics/actions/workflows/cmake.yml/badge.svg)

Forward and inverse kinematics for the 6DoF, 6-UPU class of parallel manipulators referred to as the *Stewart-Gough platform*, or as *hexapods*.

Kinematics implementation derived from [LinuxCNC's `genhexkins`](https://github.com/LinuxCNC/linuxcnc/blob/v2.8.2/src/emc/kinematics/genhexkins.c) .


## Installation
Requires [Eigen 3](https://gitlab.com/libeigen/eigen).

## Usage

See `test/test_basic.cpp` for examples.
See `test/test_kinematics.cpp` for examples.
