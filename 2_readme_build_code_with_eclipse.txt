C/C++ Programming: with Eclipse IDE
See instructions below how to install and use it Eclipse. Note that the toolchain and the libraires must be already installed prior to this.The instructions are built upon the steps from the Khepera IV user guide chapter 5.2.2.2: Using Eclipse IDE. While the maual gets most steps right, apparently it was written for a slightly older version of the IDE and some instructions are not clear enough are missing a step.

STEPS:
=====

1. Java Runtime Environment (JRE) is installed all lab computers, install JRE if not already installed with these commands:

sudo add-apt-repository ppa:webupd8team/java
sudo apt-get update
sudo apt-get install oracle-java8-installer

2. Download the linux version of "Eclipse IDE for C/C++ Developers (includes Incubating components)" from (for Ubuntu users, don't install with apt-get because, the apt-get version is older):
http://www.eclipse.org/downloads/packages/eclipse-ide-cc-developers/mars2

Note: Recent versions of the Eclipse C/C++ IDE must be usable, but the settings menu and options might be a little different and may need a littile digging.

3. Extract the Eclipse program file:

tar -xzf eclipse-cpp-mars-2-linux-gtk.tar.gz –C ~/
You can also create a link to the start file:
sudo ln -s ~/eclipse/eclipse /usr/bin/eclipse

4. You should have already installed the latest version of the khepera toolchain following the instructions from the previous guide. Light toolchain is good enough, full-toolchain is required only if you want to make kernel
modifications on the khepera. You can check the toolchain by opening a terminal and with the command (one line):

/usr/local/khepera4-yocto/build/tmp/sysroots/i686-linux/usr/bin/armv7a-vfp-neon-poky-linux-gnueabi/arm-poky-linux-gnueabi-gdb --version

If the tool chain is installed right, this should return: GNU gdb (GDB) 7.6.2

5. Run eclipse and at the "Workspace launcher" window, choose where you would like to put your project. Then close Welcome window.
Go to file menu “File => C Project” or C++; for running a C++ on the robot and choose a Project Name (ex: test). Then push next button.

6. The following steps are important, please make sure to get the paths, flags etc. correct.
In the next window "C Project", press the "Advanced Settings" button and on the “C/C++ Build => Settings”,
In “Cross Settings”: enter:

Prefix: arm-poky-linux-gnueabi-
Path: /opt/poky/1.8/sysroots/i686-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/

7. On “Cross GCC Compiler” Includes => Include paths, add:
/opt/poky/1.8/sysroots/cortexa8hf-vfp-neon-poky-linux-gnueabi/usr/include

8. On Miscellaneous, replace "Other flags" with
-c -march=armv7-a -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 --sysroot=/opt/poky/1.8/sysroots/cortexa8hf-vfp-neon-poky-linux-gnueabi
Khepera IV User Manual ver 3.0 39

9. On “the Cross GCC Linker”, on Miscellaneous, replace “Linkers flags” with:
-march=armv7-a -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 --sysroot=/opt/poky/1.8/sysroots/cortexa8hf-vfp-neon-poky-linux-gnueabi

10.On “Cross GCC Linker => Libraries” at "Libraries (-l), add khepera with the + button on the upper right

11.At "Libraries search path", add :
/opt/poky/1.8/sysroots/cortexa8hf-vfp-neon-poky-linux-gnueabi/usr/lib

12.Choose “C/C++” Build menu at the left. Choose Release under the configuration list on the right. Repeat the steps above for this configuration from 6.

13. Press “Finish” button.

14. Go to menu “File => New => Source file” choose test.c as filename

15. Close the Welcome window with its cross at the upper left.

16. Insert in the test.c file the following C source code(code does nothing, just shows the library is installed and all project setings are good):

#include <khepera/khepera.h>
int main(int argc, char *argv[]) {
int rc;
/* Set the libkhepera debug level - Highly recommended for development. */
kb_set_debug_level(2);
printf("LibKhepera Template Program\r\n");
/* Init the khepera library */
if((rc = kb_init( argc , argv )) < 0 )
return 1;
/* ADD YOUR CODE HERE */
return 0;
}

17.Then cross-compile the project with menu “Menu Project => Build-All”
=> The output file will be in the subdirectory Debug or Release of the project

18. Transfer the file "test" to your robot. You can use blutooth or serial cable or Wifi(simplest).  Currnetly the robots are configured to automatically connect to a wifi network named(ssid) "khepkhep" and wpa-password "khepkhep".See chapter 5.2.2.3 in the manual for steps on connectivity. Wifi is easier, you could either use the "scp" command to copy or even easier to use 'Filezilla' app.

19. Note execute the program file on the robot with command: ./test

Note: Once this step and the following steps are successful, you can either start writing your own robot programs or copy paste the robot code and build it.
