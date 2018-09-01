# A-Cooperative-Object-Transport-System-With-Behavior-Based-Robots
Master's thesis

For more info on the research and videos:
http://hdl.handle.net/1969.6/31364


About:
The project is to move an box (rectangular cuboid) from one location to another using robots. The challenge is that the robots cannot see the goal location as the object hides their camera. There is an overhead camera known as the Observer can assist the robots by sharing simple information bout the global view. The language used is C++. The Observer program is a standalone application that runs on the PC. It captures a frame, does color blob detection to locate the object and the goal and writes those points to a shared memory. The two robots run their own programs and they read the coordinates from the shared memory(shared memory is a text file located on a third robot) whenver required.  



Hardware
Robots      :   Khepera 4
Observer    :   Generic USB webcam + Linux PC

Software & Libraries
IDE         :   Eclipse Mars 2 C++ IDE (Refer Khepera IV userguide)
Libkhepera  :   v2.1
OpenCV 3.3  :   for both observer & robots (need cross compile for arm using the given toolchain, refer Khepera IV userguide)

Refer to the Khepera IV user manual for installing the libraries, setting up the IDE etc.

NOTE: The instructions for the setting up the environment and the links for downloading the required tools can found in the readme files 1_readme_installing_toolchain_and_library.txt and 2_readme_build_code_with_eclipse.txt.
Robots have less memory and installing OpenCV on robots is wasteful. So OpenCV was cross-compiled for arm and only the required .so files were copied to the /usr/lib folder on the robots.
List of .so files:
    libopencv_core.so.3.3
    libopencv_features2d.so.3.3
    libopencv_flann.so.3.3
    libopencv_highgui.so.3.3
    libopencv_imgcodecs.so.3.3
    libopencv_imgproc.so.3.3
    libopencv_videoio.so.3.3
