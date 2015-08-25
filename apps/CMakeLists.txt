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

CONFIGURE_OSPRAY()

# ------------------------------------------------------------
ADD_EXECUTABLE(ospPartiKD
  PartiKD.cpp
  ParticleModel.cpp
  #importers
  ImportUIntah.cpp
  ImportCOSMOS.cpp
  ImportXYZ.cpp
  ImportCosmicWeb.cpp
  ImportLAS.cpp
  )

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
FIND_PACKAGE(LAS REQUIRED)
# It seems this lets it find the modern Boost libraries, since now it defaults to
# multithreaded libs but doesn't include the -mt suffix. Maybe since this is an old
# CMake version (2.6) it's not updated to recognize this change?
set(Boost_USE_MULTITHREADED OFF)
FIND_PACKAGE(Boost 1.53 COMPONENTS program_options thread system iostreams filesystem REQUIRED)
IF(NOT LAS_FOUND)
	MESSAGE(FATAL_ERROR "Could not find libLAS (did you set $LAS_DIR if it's somewhere non-standard?)")
ENDIF()
INCLUDE_DIRECTORIES(${LAS_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

SET(LIBS ${LIBS} ${LAS_LIBRARY} ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(ospPartiKD ${LIBS} 
  ospray_xml${OSPRAY_LIB_SUFFIX})
# ------------------------------------------------------------

# ------------------------------------------------------------
# for scene graph includes
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/apps/qtViewer)
ADD_EXECUTABLE(ospPkd2Raw 
  pkd2volume.cpp
  )
TARGET_LINK_LIBRARIES(ospPkd2Raw ospray_sg_pkd${OSPRAY_LIB_SUFFIX}  
  )
# ------------------------------------------------------------
