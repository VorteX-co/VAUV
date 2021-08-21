#!/bin/bash
echo "-------------------------------------------------------------------"
echo "                     ~~~> Enter pkg name  <~~~                                   "
echo "-------------------------------------------------------------------"


read pkg

echo "==================================================================="
echo " !! Make sure that the header files are in the following structure $pkg/include/$pkg/header.hpp !! "
echo "=================================================================="


echo "..................................................................% Passing Cppling"
clang-format -style=Google -i ~/vortex_ws/src/$pkg/src/*.cpp
clang-format -style=Google -i ~/vortex_ws/src/$pkg/include/$pkg/*.hpp
echo "Passing Undcrustify"
uncrustify -c ~/ament_code_style.cfg --replace  ~/vortex_ws/src/$pkg/src/*.cpp
uncrustify -c ~/ament_code_style.cfg --replace  ~/vortex_ws/src/$pkg/include/$pkg/*.hpp
echo "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo "                         Good luck                                      "
echo "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"

