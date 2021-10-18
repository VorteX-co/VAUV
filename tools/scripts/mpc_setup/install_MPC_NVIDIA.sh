#! /bin/bash

# update
sudo apt-get update

# gfortran dependency
sudo apt-get install gfortran

# get unzip
sudo apt-get install unzip

# Ipopt: get, install, unzip
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.11.zip && unzip Ipopt-3.12.11.zip && rm Ipopt-3.12.11.zip
./install_ipopt_NVIDIA.sh ./Ipopt-3.12.11
echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib' >> ~/.bashrc 

# CppAD
sudo apt-get install cppad

# Gnuplot
sudo apt-get install gnuplot

# python and matplotlib
sudo apt-get install python-matplotlib
sudo apt-get install python2.7-dev
# Install Eigen
eigen_version="3.3.7"
wget --no-check-certificate https://bitbucket.org/eigen/eigen/get/$eigen_version.tar.bz2
tar -xf $eigen_version.tar.bz2
mv eig* eigen
mkdir eigen/build_dir
cd eigen/build_dir
cmake ..
sudo make install
cd ../../
rm -rf eigen/ $eigen_version.tar.bz2

# Install Ceres
ceres_version="ceres-solver-1.14.0"
sudo apt-get -y install cmake
sudo apt-get -y install libgoogle-glog-dev
sudo apt-get -y install libatlas-base-dev
sudo apt-get -y install libsuitesparse-dev
wget http://ceres-solver.org/$ceres_version.tar.gz
tar zxf $ceres_version.tar.gz
rm $ceres_version.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../$ceres_version
make
sudo make install
cd ..
rm -rf $ceres_version/ ceres-bin/


