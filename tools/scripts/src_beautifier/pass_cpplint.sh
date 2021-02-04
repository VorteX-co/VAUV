#!/bin/bash

echo " ~~~> formatting all your C++ source code to Google C++ style <~~~ " 

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




##  ------------------------------------------------------------------------ ##
# Linting and automatic formatting of every c++ file inside the vortex_ws to Google C++ style
# uncomment the following commands for reformatting all the source code in vortex_ws for you
##  ------------------------------------------------------------------------ ##

#clang-format -style=Google -i ~/vortex_ws/src/*/src/*/*.cpp
  
### Assuming that each c++ pkg contains src/<pkg_name>/.cpp files ###


#clang-format -style=Google -i ~/vortex_ws/src/*/include/*/*.hpp 

### Assuming that each c++ pkg contains include/<pkg_name>/.hpp headers ###

##  ------------------------------------------------------------------------ ##
# You can use clang-format -style=Google -i *.cpp *.hpp in any directory you want
##  ----------------------------------------------------------------------------------------- ##
