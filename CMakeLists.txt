## ======================================================================== ##
## Copyright 2009-2014 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

# mini-scene graph viewer for _geometry_ (intentionally simple)
OPTION(OSPRAY_MODULE_PKD "Build Particle KD Tree Module." ON)
OPTION(OSPRAY_MODULE_PKD_BUILDER "Build Particle KD Tree Builder apps." ON)
OPTION(OSPRAY_MODULE_PKD_SG "Build Particle KD Tree Scenegraph component." ON)
OPTION(OSPRAY_MODULE_PKD_LIDAR "Build Particle KD Tree Module for LiDAR (requires LAStools)." OFF)

IF (OSPRAY_MODULE_PKD)
  IF (OSPRAY_MODULE_PKD_LIDAR)
    SET(PKD_LIDAR_ENABLED 1)
  ELSE()
    SET(PKD_LIDAR_ENABLED 0)
  ENDIF()
  CONFIGURE_FILE("PKDConfig.h.in" PKDConfig.h)
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/apps/common/)
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/)
  INCLUDE_DIRECTORIES_ISPC(${CMAKE_CURRENT_BINARY_DIR})

  IF (OSPRAY_MODULE_PKD_SG)
    set(SG_SRCS "sg/PKD.cpp")
  ENDIF()

  # ------------------------------------------------------------
  OSPRAY_CREATE_LIBRARY(ospray_module_pkd
    ospray/PKDGeometry.cpp
    ospray/PKDGeometry.ispc
    ospray/MinMaxBVH2.cpp
    ospray/MinMaxBVH2.ispc
    #ospray/AlphaSpheres.cpp
    #ospray/AlphaSpheres.ispc
    ospray/TraversePacket.ispc
    ospray/TraverseSPMD.ispc

    #ospray/render/PKDSplatter.ispc
    #ospray/render/PKDSplatter.cpp

    ${SG_SRCS}

  LINK
    ospray
    ospray_sg
    ospray_common
    ospray_module_ispc
  )
  # ------------------------------------------------------------

  # the scene graph plugin
  #ADD_SUBDIRECTORY(sg)

  # helper apps (converters, builders,...)
  IF (OSPRAY_MODULE_PKD_BUILDER)
    ADD_SUBDIRECTORY(apps)
  ENDIF()

#  CONFIGURE_MPI()
#  ADD_EXECUTABLE(ospDPKDservice
#    dPKD/service.cpp
#    )
#  TARGET_LINK_LIBRARIES(ospDPKDservice ospray_xml${OSPRAY_LIB_SUFFIX})

ENDIF()
