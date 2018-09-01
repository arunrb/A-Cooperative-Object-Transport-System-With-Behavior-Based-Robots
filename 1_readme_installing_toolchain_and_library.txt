INSTALLING THE TOOLCHAIN AND LIBRARY:
=====================================
This guide is for the priliminary steps that must be done prior to installing the Eclipse IDE or building the project. The guide is build on top of section 5 of the Khepera IV user guide.

Read this first:
The installation of the software required to use the board on the robot and the development tool is described here. Plese read all the instructions and understand the the process before proceeding to actual installation. The robot uses an arm chip and thus all code you write on other platforms, say your PC with x86 must be cross-compiled to be run on the robot. This corss-compilation is done using the toolchain. Khepera has 2 toolchains: light and full. Light chain is what you would need most of the time, full toolchain and sources are for advanced users requiring kernel modifications(on the yocto linux), packages creation/addition and so on.

Listed below are the files needed to install the light toolchain. These files are available from the Khepera IV's  URL: http://ftp.k-team.com/KheperaIV/software/
After downloading the package, the folder would have the following:
a. From light_toolchain/ folder:
    Light toolchain (script for installing cross-compiler):
        poky-glibc-i686-khepera4-image-cortexa8hf-vfp-neon-toolchain-1.8.sh
b. From library/ folder:
        Board library sources: libkhepera_2.X.tar.bz2 *
        * where X is the release version (may change without notice)

Steps for installing the light toolchain:
========================================
1. Create the development directory
The development directory will be the base folder for your development. It contains links and scripts to easily use the cross-compiler to make your programs.
Create a new development folder ~/khepera4_development in your home directory and enter into it. You can use the following commands, assuming you are in a console.

mkdir ~/khepera4_development
cd ~/khepera4_development

Installation of the cross-compiler (light toolchain)
====================================================
1. Install the cross-compiler either in /opt or in your own account if you don't have root access:

chmod +x poky-glibc-i686-khepera4-image-cortexa8hf-vfp-neon-toolchain-1.8.sh
sudo ./poky-glibc-i686-khepera4-image-cortexa8hf-vfp-neon-toolchain-1.8.sh

2. Follow the instructions of the installation program using default settings (or choosing a folder that you have writing rights if you don't have root access).
You can check if the installation is correct by running the cross-compiler. Firstly make the environment variables available (to be done every time you open a new terminal to cross-compile a program):

source /opt/poky/1.8/environment-setup-cortexa8hf-vfp-neon-poky-linux-gnueabi

3.To check the version of the cross-compiler:

arm-poky-linux-gnueabi-gcc --version

This command should return:
arm-poky-linux-gnueabi-gcc (GCC) 4.9.2


Installation of the robot library
=================================
The library is already installed on the robot, so don't worry about. Library needs to be installed library on your development system, follow the following instructions:

1. Extract the library libkhepera_2.X.tar.bz2 in your development folder:

tar -xjf libkhepera_2.X.tar.bz2 -C ~/khepera4_development

2. You can recompile the whole library by running the following commands in the libkhepera-2.X folder:

cd ~/khepera4_development/libkhepera-2.X
make clean
make all

Note 1:
If you modified the library (any file in src/), you will have to transfer the file build-khepera-3.18.18-custom/lib/libkhepera.so.2.X to your robot, overwriting /usr/lib/libkhepera.so.2.X. This would not be necessary if you are following this guide as you would be using the library as it is.

Note 2:
The board library contains these files and directories:
    1. build-khepera-3.18.18-custom/  = > compiled library and headers
    
    2. doc/ documentation => (API: start in doc/html/index.html or simply open any html file in this directory, it would be oopened on your web-browser where you would be able to navigate to all the libray components, data-structures, api's etc.)
    
    3. src/  => source code of the library
    
    4. src/tests => examples and tests source code. This is a good place to start. Pretty much everything you would need to understand about the robot and the library  functionality could be found here.. 
    
    5. template/ template program
    
    6. Makefile Makefile for all
    
    7. README.kteam readme file => good place to start for understnding the organization of the contents and stuff.

Note 3:
You can find an updated version of the library from the following FTP site:
http://ftp.k-team.com/KheperaIV/software/library/
