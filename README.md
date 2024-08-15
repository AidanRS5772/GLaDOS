PROJECT STILL UNDER CONSTRUCTION

Setup Guide For Client:

REQUIRMENTS:

  - Have conan2 version 2.4 or higher installed and added to PATH (It is not autoumatically added to PATH when installing alot of the time)
  - Have Cmake version 3.24 or higher installed and added to PATH
  - Setup conan default profile on system. To setup run the following in the home directory.
    ```
    conan profile detect --force
    ```
    Then you are going to want to add the following to the bottom of the .conan2/profiles/default file with your perfered text editor.
    ```
    [conf]
    tools.system.package_manager:mode=install
    tools.system.package_manager:sudo = True
    ```
  - Be proficent in C++, CMake, Conan2, and Python

Step 1:

All packages,linkers, and generators will be held in a build directory. Once you have downloaded the repo in the GLaDOS Folder run the comand below. Conan will create the make folder if it doesn't exist and it will modify it does exist or you have already made it.
```
conan install . -of=build --build=missing
```
This will create the build folder which will have the following structer
```
.
|__build
    |__build
        |__Release
            |__(other files)
```
(redundent I know, but nessacary for cross platform compatability)
This will also produce a CMakeUserPreset.json.

Step 2:

In the top level build directory run the following Cmake command.

```
cmake .. --preset conan-release
```

This will produce the proper Makefile. This comand will fail if you have any cmake version less then 3.24.

Step 3:

In the Release folder of build run the following command
```
make
```
this will generate the executable GLaDOS in the same folder.

Step 4:

Run the executable with...
```
./GLaDOS
```

Any time you want to change the build system you need rerun steps 2 through 4, and anytime you want to add or remove a package from the dependencies you need rerun all the steps. It would be benificial to familiarize yourself with how to use Conan and Cmake as well. If you do your code editing in vscode you may want to add "~/.conan2/p/**" to the "includePath" section of the "c_cpp_properties.json" file to have intelisense be able to see the packages downloaded from conan.

Setup Guide for Server:

REQUIRMENTS:

  - Have latest python interpreter installed
  - Have latest pip installed

Step 1:

Create the vitual enviorment.
```
python -m venv env
source env/bin/activate
```

Step 2:

Download the proper packages.
```
pip install numpy opencv-python
```

Step 3:
Run server code.
```
cd src
python py_server.py
```
