# Install script for directory: /home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/build/libsoem.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/soem/cmake/soemConfig.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/soem/cmake/soemConfig.cmake"
         "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/build/CMakeFiles/Export/39806c66e6e7fd9076eb39407f12ee6f/soemConfig.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/soem/cmake/soemConfig-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/soem/cmake/soemConfig.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/soem/cmake" TYPE FILE FILES "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/build/CMakeFiles/Export/39806c66e6e7fd9076eb39407f12ee6f/soemConfig.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/soem/cmake" TYPE FILE FILES "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/build/CMakeFiles/Export/39806c66e6e7fd9076eb39407f12ee6f/soemConfig-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/soem" TYPE FILE FILES
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/soem/ethercat.h"
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/soem/ethercatbase.h"
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/soem/ethercatcoe.h"
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/soem/ethercatconfig.h"
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/soem/ethercatconfiglist.h"
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/soem/ethercatdc.h"
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/soem/ethercateoe.h"
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/soem/ethercatfoe.h"
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/soem/ethercatmain.h"
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/soem/ethercatprint.h"
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/soem/ethercatsoe.h"
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/soem/ethercattype.h"
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/osal/linux/osal_defs.h"
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/osal/osal.h"
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/oshw/linux/nicdrv.h"
    "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/oshw/linux/oshw.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/build/test/simple_ng/cmake_install.cmake")
  include("/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/build/test/linux/slaveinfo/cmake_install.cmake")
  include("/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/build/test/linux/eepromtool/cmake_install.cmake")
  include("/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/build/test/linux/simple_test/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/burb/elfin_s_controlbox/src/elfin_s_robot/SOEM/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
