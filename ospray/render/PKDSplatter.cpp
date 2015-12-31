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
#include "ospray/render/Renderer.h"
#include "ospray/camera/PerspectiveCamera.h"
// ispc exports
#include "PKDSplatter_ispc.h"
// this module
#include "../PKDGeometry.h"

namespace ospray {
  namespace pkd {
    struct PKDSplatter : public Renderer {
      PKDSplatter();
      virtual std::string toString() const { return "ospray::pkd::PKDSplatter"; }
      
      Model  *model;
      Camera *camera;
      float splatRadius;
      float splatWeight;

      virtual void commit();
    };
    
    PKDSplatter::PKDSplatter()
      : model(NULL), camera(NULL) 
    {
      ispcEquivalent = ispc::PKDSplatter_create(this);
   }
    
    void PKDSplatter::commit()
    {
      Renderer::commit();
      
      model = (Model *)getParamObject("world",NULL);
      model = (Model *)getParamObject("model",model);
      camera = (Camera *)getParamObject("camera",NULL);
      splatWeight = getParamf("weight",.0001f);
      splatRadius = getParamf("radius",1.f);

      if (!model) return;
      assert(model->geometry.size() == 1);
      PartiKDGeometry *pkd = dynamic_cast<PartiKDGeometry *>(model->geometry[0].ptr);
      assert(pkd);

      ispc::PKDSplatter_set(getIE(),
                            model?model->getIE():NULL,
                            pkd?pkd->getIE():NULL,
                            camera?camera->getIE():NULL,
                            splatRadius,splatWeight);
    }
    
    OSP_REGISTER_RENDERER(PKDSplatter,pkd_splatter);
  } // ::ospray::pkd
} // ::ospray



