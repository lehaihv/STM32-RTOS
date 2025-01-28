# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/CM4"
  "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/CM4/build"
  "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/CM4"
  "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/CM4/tmp"
  "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/CM4/src/App_1_CM4-stamp"
  "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/CM4/src"
  "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/CM4/src/App_1_CM4-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/CM4/src/App_1_CM4-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/CM4/src/App_1_CM4-stamp${cfgdir}") # cfgdir has leading slash
endif()
