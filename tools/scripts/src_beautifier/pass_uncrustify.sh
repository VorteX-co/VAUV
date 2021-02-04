#!/bin/bash

##--------------------------------------------------------------------------------##
# Installing uncrustify tool
##-------------------------------------------------------------------------------##

sudo apt-get install uncrustify

##--------------------------------------------------------------------------------##
# Downloading the configuration file in order to pass ament_uncrustify test
##-------------------------------------------------------------------------------##

cd ~
curl -LJO https://raw.githubusercontent.com/ament/ament_lint/master/ament_uncrustify/ament_uncrustify/configuration/ament_code_style.cfg

##--------------------------------------------------------------------------------##
# 
# uncomment the following commands for reformatting all your c++ code
# to  ament_uncrustify_lint style.
##-------------------------------------------------------------------------------##
#uncrustify -c ~/ament_code_style.cfg --replace  ~/vortex_ws/src/*/src/*/*.cpp
#uncrustify -c ~/ament_code_style.cfg --replace  ~/vortex_ws/src/*/include/*/*.hpp


