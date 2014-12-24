// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "PKDGeometry.h"
// ispc exports
#include "PKDGeometry_ispc.h"

namespace ospray {

  /*! \brief integrates this geometry's primitives into the respective
    model's acceleration structure */
  void PartiKDGeometry::finalize(Model *model) 
  {
    PING;
  }    

  OSP_REGISTER_GEOMETRY(PartiKDGeometry,partikd_geometry);

} // ::ospray
