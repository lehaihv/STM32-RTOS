# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/CM7"
  "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/CM7/build"
  "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/build/Debug/CM7"
  "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/build/Debug/CM7/tmp"
  "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/build/Debug/CM7/src/App_1_CM7-stamp"
  "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/build/Debug/CM7/src"
  "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/build/Debug/CM7/src/App_1_CM7-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/build/Debug/CM7/src/App_1_CM7-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/GitHub/STM32-RTOS/UF2024/Nucleo_H755ZI/New Folder/App_1/build/Debug/CM7/src/App_1_CM7-stamp${cfgdir}") # cfgdir has leading slash
endif()
