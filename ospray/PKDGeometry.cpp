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
// ospray
#include "ospray/common/Model.h"
// ispc exports
#include "PKDGeometry_ispc.h"

namespace ospray {
  using std::endl;
  using std::cout;

  //! Constructor
  PartiKDGeometry::PartiKDGeometry()
    : particleRadius(.02f)
  {
    PING;
    ispcEquivalent = ispc::PartiKDGeometry_create(this);
  }

  /*! return bounding box of particle centers */
  box3f PartiKDGeometry::getBounds() const
  {
    box3f b = empty;
    for (size_t i=0;i<numParticles;i++)
      b.extend(particle[i]);
    return b;
  }
  /*! \brief integrates this geometry's primitives into the respective
    model's acceleration structure */
  void PartiKDGeometry::finalize(Model *model) 
  {
    // -------------------------------------------------------
    // parse parameters, using hard assertions (exceptions) for now.
    //
    // note:
    // - "float radius" *MUST* be defined with the object
    // - "data<vec3f> particles' *MUST* be defined for the object
    // -------------------------------------------------------
    particleData = getParamData("particles");
    if (!particleData)
      throw std::runtime_error("#osp:pkd: no 'particles' data found with object");

    particle     = (vec3f*)particleData->data;
    numParticles = particleData->numItems;
    const box3f centerBounds = getBounds();
    
    particleRadius = getParamf("radius",0.f);
    if (particleRadius <= 0.f)
      throw std::runtime_error("#osp:pkd: invalid radius (<= 0.f)");
    const float expectedRadius
      = (centerBounds.size().x+centerBounds.size().y+centerBounds.size().z)*powf(numParticles,1.f/3.f);
    if (particleRadius > 10.f*expectedRadius)
      cout << "#osp:pkd: Warning - particle radius is pretty big for given particle configuration !?" << endl;
    
    const box3f sphereBounds(centerBounds.lower - vec3f(particleRadius),
                             centerBounds.upper + vec3f(particleRadius));
    size_t numInnerNodes = numParticles/2;

    // -------------------------------------------------------
    // actually create the ISPC-side geometry now
    // -------------------------------------------------------
    ispc::PartiKDGeometry_set(getIE(),model->getIE(),
                              transferFunction?transferFunction->getIE():NULL,
                              particleRadius,
                              (ispc::PKDParticle*)particle,
                              numParticles,
                              NULL,
                              numInnerNodes,
                              (ispc::box3f&)centerBounds,(ispc::box3f&)sphereBounds,
                              0.f,0.f);

  }    

  OSP_REGISTER_GEOMETRY(PartiKDGeometry,pkd_geometry);

} // ::ospray

extern "C" void ospray_init_module_pkd() 
{
  std::cout << "#osp:pkd: loading 'pkd' module" << std::endl;
}
