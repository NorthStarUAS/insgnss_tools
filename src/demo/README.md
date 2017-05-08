These demo apps show how to setup and call the EKF.  The following
compile commands are example and will very likely need to be adjusted
for your own system.  (I am keeping things simple and avoiding
makefiles for now ...)


# 15 State Inertial Only EKF

## Compile with native compiler

g++ -O3 -I../nav_eigen ../nav_eigen/EKF_15state.cxx ../core/nav_functions.cxx ekf15_demo.cpp -o ekf15_demo -lm -lrt


## Compile with cross compiler

/opt/codesourcery/arm-2009q1/bin/arm-none-linux-gnueabi-g++ -I/home/sentera/AuraUAS -I../nav_eigen -O3 -mfpu=vfp -ftree-vectorize -mfloat-abi=softfp ../nav_eigen/EKF_15state.cxx ekf15_demo.cpp ../core/nav_functions.cxx -o ekf15_demo -lm -lrt


# 15 State EKF with magnetometer measurement correction

## Compile with native compiler

g++ -O3 -I../nav_eigen_mag ../nav_eigen_mag/EKF_15state_mag.cxx ../core/nav_functions.cxx ../core/coremag.c linearfit.cxx ekf15_mag_demo.cpp -o ekf15_mag_demo -lm -lrt


## Compile with cross compiler

/opt/codesourcery/arm-2009q1/bin/arm-none-linux-gnueabi-g++ -I/home/sentera/AuraUAS -I../nav_eigen_mag -O3 -mfpu=vfp -ftree-vectorize -mfloat-abi=softfp ../nav_eigen_mag/EKF_15state_mag.cxx linearfit.cxx ekf15_mag_demo.cpp ../core/nav_functions.cxx ../core/coremag.c -o ekf15_mag_demo -lm -lrt


## As a static library

/opt/codesourcery/arm-2009q1/bin/arm-none-linux-gnueabi-g++ -I/home/sentera/Source/AuraUAS -I../nav_eigen_mag -O3 -mfpu=vfp -ftree-vectorize -mfloat-abi=softfp -c ../nav_eigen_mag/EKF_15state_mag.cxx

/opt/codesourcery/arm-2009q1/bin/arm-none-linux-gnueabi-g++ -I/home/sentera/Source/AuraUAS -I../nav_eigen_mag -O3 -mfpu=vfp -ftree-vectorize -mfloat-abi=softfp -c ../core/nav_functions.cxx

/opt/codesourcery/arm-2009q1/bin/arm-none-linux-gnueabi-g++ -I/home/sentera/Source/AuraUAS -I../nav_eigen_mag -O3 -mfpu=vfp -ftree-vectorize -mfloat-abi=softfp -c ../core/coremag.c 

ar rcs libEKFmag.a EKF_15state_mag.o nav_functions.o coremag.o 
