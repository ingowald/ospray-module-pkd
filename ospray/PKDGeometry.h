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

#pragma once

#include "ospray/geometry/Geometry.h"
#include "ospray/common/Data.h"
#include "ospray/transferFunction/TransferFunction.h"

namespace ospray {

  /*! the actual ospray geometry for a PartiKD */
  struct PartiKDGeometry : public ospray::Geometry {
    //! Constructor
    PartiKDGeometry();

    //! \brief common function to help printf-debugging 
    virtual std::string toString() const { return "ospray::PartiKDGeometry"; }

    /*! \brief integrates this geometry's primitives into the respective
      model's acceleration structure */
    virtual void finalize(Model *model);

    /*! return bounding box of particle centers */
    box3f getBounds() const;

    /*! gets called whenever any of this node's dependencies got changed */
    virtual void dependencyGotChanged(ManagedObject *object);

    //! transfer function for color/alpha mapping, may be NULL
    Ref<TransferFunction> transferFunction;
    Ref<Data> particleData;
    Ref<Data> attributeData;

    float    *attribute;
    vec3f    *particle;
    size_t    numParticles;
    float     particleRadius;
  };
  
} // ::ospray
