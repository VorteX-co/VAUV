#!/bin/bash

echo " ~~~> Installing some softwares <~~~ " 

##  ------------------------------------------------------------------------ ##
# Requirements installation & downloading Homebrew
##  ------------------------------------------------------------------------ ##

sudo apt-get install build-essential curl file git
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

##  ------------------------------------------------------------------------ ##
#installs Homebrew to /home/linuxbrew/.linuxbrew using sudo if possible and in your home directory at ~/.linuxbrew otherwise.
##  ------------------------------------------------------------------------ ##

test -d ~/.linuxbrew && eval $(~/.linuxbrew/bin/brew shellenv)
test -d /home/linuxbrew/.linuxbrew && eval $(/home/linuxbrew/.linuxbrew/bin/brew shellenv)
echo "eval \$($(brew --prefix)/bin/brew shellenv)" >>~/.profile

##  ------------------------------------------------------------------------ ##
# installing clang-format via Homebrew
##  ------------------------------------------------------------------------ ##

brew install clang-format


##--------------------------------------------------------------------------------##
# Installing uncrustify tool
##-------------------------------------------------------------------------------##

sudo apt-get install uncrustify

##--------------------------------------------------------------------------------##
# Downloading the configuration file in order to pass ament_uncrustify test
##-------------------------------------------------------------------------------##

cd ~
curl -LJO https://raw.githubusercontent.com/ament/ament_lint/master/ament_uncrustify/ament_uncrustify/configuration/ament_code_style.cfg

echo " --------------------------------------------------------"
echo " Done  "
echo " --------------------------------------------------------"




