# K-D Tree

K-D Tree is a C++11 library for solving the nearest neighbor problem.

## Getting Started

These instructions will help you generate the necessary documentation for using this library, and list the required dependencies.

### Documentation

The documentation for this project is Doxygen based. To generate, execute the following commands:

```
cd <path>/kd_tree
doxygen Doxyfile
```

### Dependencies

The follwing dependencies are required, and can be installed accordingly.

```
sudo apt install doxygen
sudo apt install build-essential
sudo apt install python-catkin-tools
sudo apt install libgtest-dev

```

## Running the tests

To run the unit tests for this package, use the following command:

```
catkin build kd_tree --no-deps --catkin-make-args run_tests
```

### Break down into end to end tests

The unit tests for this package are split into three groups, State, Node, and KdTree. Each test file executes a series of unit tests to validate template class functionality. 

```
kd_tree_test.cpp
node_test.cpp
state_test.cpp
```

## Built With

* [Google Test](https://github.com/google/googletest) - The unit testing framework used
* [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/index.html) - The build tool used in compiling this project

## Authors

* **Sean Crutchlow**

## License

This project is licensed under the MIT License - see the LICENSE file for details
