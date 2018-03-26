#!/bin/sh

#SeedUpperBodyRTCのbuild
cd ../RTC/SeedUpperBodyRTC/
mkdir build
cd build
cmake ..
make

#SeedUpperBodyControllerのbuild
cd ../../SeedUpperBodyController/
mkdir build
cd build
cmake ..
make

#IkSolvers_SeedWaistRightArmのbuild
cd ../../IkSolvers_SeedWaistRightArm/
mkdir build
cd build
cmake ..
make

#IkSolvers_SeedWaistLeftArmのbuild
cd ../../IkSolvers_SeedWaistLeftArm/
mkdir build
cd build
cmake ..
make

#元のディレクトリへ
cd ../../script
