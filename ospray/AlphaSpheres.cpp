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

#undef NDEBUG

// ospray
#include "AlphaSpheres.h"
#include "ospray/common/Data.h"
#include "ospray/common/Model.h"
// ispc-generated files
#include "AlphaSpheres_ispc.h"
//#include "PartiKD.h"

namespace ospray {
  using std::cout;
  using std::endl;

  AlphaSpheres::AlphaSpheres()
  {
    this->ispcEquivalent = ispc::AlphaSpheres_create(this);
  }

  void AlphaSpheres::buildBVH() 
  {
    PrimAbstraction pa(this);
    mmBVH.initialBuild(&pa);
  }
  
  void AlphaSpheres::finalize(Model *model) 
  {
    radius            = getParam1f("radius",0.01f);
    attributeData     = getParamData("attribute",NULL);
    positionData      = getParamData("position",NULL);
    transferFunction  = (TransferFunction *)getParamObject("transferFunction",NULL);
    
    if (!positionData) 
      throw std::runtime_error("#osp:AlphaParticless: no 'particles' data specified");
    // if (attributeData == NULL) 
    //   throw std::runtime_error("#osp:AlphaAttributes: no 'attribute' data specified");
    numSpheres = positionData->numBytes / sizeof(vec3f);

    std::cout << "#osp: creating 'alpha_spheres' geometry, #spheres = " << numSpheres << std::endl;
    
    if (numSpheres >= (1ULL << 30)) {
      throw std::runtime_error("#ospray::Spheres: too many spheres in this sphere geometry. Consider splitting this geometry in multiple geometries with fewer spheres (you can still put all those geometries into a single model, but you can't put that many spheres into a single geometry without causing address overflows)");
    }
    
    position  = (vec3f *)positionData->data;
    attribute = attributeData ? (float *)attributeData->data : NULL;

    buildBVH();

    ispc::AlphaSpheres_set(getIE(),
                           model->getIE(),
                           transferFunction?transferFunction->getIE():NULL,
                           mmBVH.rootRef,
                           mmBVH.getNodePtr(),
                           &mmBVH.primID[0],
                           radius,
                           (ispc::vec3f*)position,attribute,
                           numSpheres);

    PRINT(mmBVH.node.size());
    PRINT(mmBVH.node[0]);
  }


  OSP_REGISTER_GEOMETRY(AlphaSpheres,alpha_spheres);

}
