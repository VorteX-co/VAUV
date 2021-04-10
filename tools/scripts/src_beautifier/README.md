
# C++ source code beautifier

## Description

The src_beautifier tool is a collection of commands for reformatting C++ source code. Specifically design for passing ament_cmake tests and lints.

**Author: vortex-co<br />
Affiliation: [VorteX-Co](https://vortex-co.com/home)<br />
Maintainer: vortex-co, info@vortex-co.com**


## Table of content
* [Ament Cmake lints](#Ament-Cmake-lints)
* [Specification](#Specification)
* [Usage](#Usage)
* [Video](#Video)

## Ament-Cmake-lints

Build test mechanism invoked from CmakeLists.txt within a C++ ROS2 package. The test compares the source code style/format against some common linters, the set of linters available are:

* copyright   ~> a copyright linter which checks that copyright statements and license headers are present and correct
* cppcheck    ~> a C++ checker which can also find some logic tests
* cpplint     ~> a C++ style checker (e.g. comment style)
* xmllint     ~> an xml linter
* uncrustify  ~> a C++ style checker
* lint_camke  ~> a cmake linter


## Specification

This tool contains two scripts, one for setup some software packages and the other asks the user to enter a package name then it parses the .cpp/.hpp files for passing the tests.
You should add the copyright and lisecne header manually.

List of lint-errors that cannot be solved by this tool:

* Using deprecated casting style. ~~~> Don't use C-style for casting.
* Wrong order of the included headers (The following order is the correct 1-corresponded header file 2- C headers 3- c++ headers)
* Missed 'extern' keyword in front of a single-paramter class constructor.
* Missed copyright message/header in the source code.  ~~~> add the comments inside example_license.txt file at the starting of your code.
* Using STL features (vector,Matrix,queue) without including the respective C++ header.
* Using C data types like 'long x' ~> instead use 'uint64_t x'.
* Missed #ifndef header guard inside a C++ header file (.hpp). 


## Usage 

### Setup script
```
$ sudo ./ubuntu_setup.sh

```
### Cleaning C++ package

```
$ sudo ./clean_pkg.sh
```
Then you should enter the name of the C++ package for formatting.

### Note

The following structure of files is assumed:

~/vortex_ws/src/PACKAGE_NAME/

|--include
    |--(PACKAGE_NAME)
       |--a_file.hpp
|--src
    |--a_file.cpp

## Video

A recorded demo video:
https://drive.google.com/drive/u/0/folders/1WXbzIly_6CDalZd49NzHAmY2H9BoaMs9
