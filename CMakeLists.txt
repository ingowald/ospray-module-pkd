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

# mini-scene graph viewer for _geometry_ (intentionally simple)
OPTION(OSPRAY_MODULE_PARTIKD "Build Particle KD Tree Module." ON)

  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ospray/include)
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/)
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

  OSPRAY_ADD_LIBRARY(ospray_module_pkd${OSPRAY_LIB_SUFFIX} SHARED
    ospray/PKDGeometry.cpp
    ospray/PKDGeometry.ispc
    ospray/MinMaxBVH2.cpp
    ospray/MinMaxBVH2.ispc
    ospray/AlphaSpheres.cpp
    ospray/AlphaSpheres.ispc
    )
  TARGET_LINK_LIBRARIES(ospray_module_pkd${OSPRAY_LIB_SUFFIX} ${LIBS})

IF (OSPRAY_MODULE_PARTIKD AND NOT THIS_IS_MIC)
  ADD_EXECUTABLE(ospPartiKD
    apps/PartiKD.cpp
    apps/ParticleModel.cpp
    #importers
    apps/ImportUIntah.cpp
    apps/ImportXYZ.cpp
    apps/ImportCosmicWeb.cpp
    )
  TARGET_LINK_LIBRARIES(ospPartiKD ${LIBS} ospray_xml${OSPRAY_LIB_SUFFIX}  ospray_xml${OSPRAY_LIB_SUFFIX})
  # ------------------------------------------------------------
ENDIF()
