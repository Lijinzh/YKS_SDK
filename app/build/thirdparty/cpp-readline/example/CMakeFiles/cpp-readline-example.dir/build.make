# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/gold/YKS_SDK/app

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gold/YKS_SDK/app/build

# Include any dependencies generated for this target.
include thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/depend.make

# Include the progress variables for this target.
include thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/progress.make

# Include the compile flags for this target's objects.
include thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/flags.make

thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/main.cpp.o: thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/flags.make
thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/main.cpp.o: ../thirdparty/cpp-readline/example/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gold/YKS_SDK/app/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/main.cpp.o"
	cd /home/gold/YKS_SDK/app/build/thirdparty/cpp-readline/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cpp-readline-example.dir/main.cpp.o -c /home/gold/YKS_SDK/app/thirdparty/cpp-readline/example/main.cpp

thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cpp-readline-example.dir/main.cpp.i"
	cd /home/gold/YKS_SDK/app/build/thirdparty/cpp-readline/example && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gold/YKS_SDK/app/thirdparty/cpp-readline/example/main.cpp > CMakeFiles/cpp-readline-example.dir/main.cpp.i

thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cpp-readline-example.dir/main.cpp.s"
	cd /home/gold/YKS_SDK/app/build/thirdparty/cpp-readline/example && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gold/YKS_SDK/app/thirdparty/cpp-readline/example/main.cpp -o CMakeFiles/cpp-readline-example.dir/main.cpp.s

# Object files for target cpp-readline-example
cpp__readline__example_OBJECTS = \
"CMakeFiles/cpp-readline-example.dir/main.cpp.o"

# External object files for target cpp-readline-example
cpp__readline__example_EXTERNAL_OBJECTS =

thirdparty/cpp-readline/example/cpp-readline-example: thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/main.cpp.o
thirdparty/cpp-readline/example/cpp-readline-example: thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/build.make
thirdparty/cpp-readline/example/cpp-readline-example: thirdparty/cpp-readline/src/libcpp-readline.so.0.1.0
thirdparty/cpp-readline/example/cpp-readline-example: /usr/lib/x86_64-linux-gnu/libreadline.so
thirdparty/cpp-readline/example/cpp-readline-example: thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gold/YKS_SDK/app/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable cpp-readline-example"
	cd /home/gold/YKS_SDK/app/build/thirdparty/cpp-readline/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cpp-readline-example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/build: thirdparty/cpp-readline/example/cpp-readline-example

.PHONY : thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/build

thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/clean:
	cd /home/gold/YKS_SDK/app/build/thirdparty/cpp-readline/example && $(CMAKE_COMMAND) -P CMakeFiles/cpp-readline-example.dir/cmake_clean.cmake
.PHONY : thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/clean

thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/depend:
	cd /home/gold/YKS_SDK/app/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gold/YKS_SDK/app /home/gold/YKS_SDK/app/thirdparty/cpp-readline/example /home/gold/YKS_SDK/app/build /home/gold/YKS_SDK/app/build/thirdparty/cpp-readline/example /home/gold/YKS_SDK/app/build/thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : thirdparty/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/depend

