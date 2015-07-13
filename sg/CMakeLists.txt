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
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/apps/qtViewer)

OSPRAY_ADD_LIBRARY(ospray_sg_pkd${OSPRAY_LIB_SUFFIX} SHARED
  PKD.cpp
  )
TARGET_LINK_LIBRARIES(ospray_sg_pkd${OSPRAY_LIB_SUFFIX} 
  ospray${OSPRAY_LIB_SUFFIX}
  ospray_sg${OSPRAY_LIB_SUFFIX}
  )
# ------------------------------------------------------------
