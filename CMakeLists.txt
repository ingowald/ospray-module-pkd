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
OPTION(OSPRAY_MODULE_PARTIKD "Build Particle KD Tree Module." ON)
OPTION(OSPRAY_MODULE_PARTIKD_LIDAR "Build Particle KD Tree Module for LiDAR (requires LAStools)." OFF)
OPTION(OSPRAY_MODULE_PKD_ISPV "Build PKD-module in-situ particle vis service" OFF)

IF (OSPRAY_MODULE_PARTIKD)
  IF (OSPRAY_MODULE_PARTIKD_LIDAR)
	SET(PARTIKD_LIDAR_ENABLED 1)
  ELSE()
    SET(PARTIKD_LIDAR_ENABLED 0)
  ENDIF()
  CONFIGURE_FILE("PKDConfig.h.in" PKDConfig.h)
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
  INCLUDE_DIRECTORIES_ISPC(${CMAKE_CURRENT_BINARY_DIR})
  # ------------------------------------------------------------
  OSPRAY_CREATE_LIBRARY(module_pkd
    ospray/PKDGeometry.cpp
    ospray/PKDGeometry.ispc
    ospray/MinMaxBVH2.cpp
    ospray/MinMaxBVH2.ispc
    ospray/AlphaSpheres.cpp
    ospray/AlphaSpheres.ispc
    ospray/TraversePacket.ispc
    ospray/TraverseSPMD.ispc

    ospray/render/PKDSplatter.ispc
    ospray/render/PKDSplatter.cpp
    LINK
    ospray
  )
  # ------------------------------------------------------------

  IF (NOT THIS_IS_MIC)
    # the scene graph plugin
    ADD_SUBDIRECTORY(sg)
    
    # helper apps (converters, builders,...)
    ADD_SUBDIRECTORY(apps)
  ENDIF()

#  CONFIGURE_MPI()
#  ADD_EXECUTABLE(ospDPKDservice
#    dPKD/service.cpp
#    )
#  TARGET_LINK_LIBRARIES(ospDPKDservice ospray_xml${OSPRAY_LIB_SUFFIX})

ENDIF()
