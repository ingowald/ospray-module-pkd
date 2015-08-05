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
  )
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
