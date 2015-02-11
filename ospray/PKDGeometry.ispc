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

// ospray
#include "ospray/math/vec.ih"
#include "ospray/math/bbox.ih"
#include "ospray/common/Ray.ih"
#include "ospray/geometry/Geometry.ih"
#include "ospray/common/Model.ih"
#include "ospray/transferFunction/LinearTransferFunction.ih"
// this module
#include "PKDGeometry.ih"
// embree
#include "embree2/rtcore.isph"
#include "embree2/rtcore_scene.isph"
#include "embree2/rtcore_geometry_user.isph"

#include "ospray/common/Box.ih"

static void PartiKDGeometry_postIntersect(uniform Geometry *uniform geometry,
                                          uniform Model *uniform model,
                                          varying DifferentialGeometry &dg,
                                          const varying Ray &ray,
                                          uniform int64 flags)
{
  uniform PartiKDGeometry *uniform THIS = (uniform PartiKDGeometry *uniform)geometry;
  dg.geometry = geometry;
  dg.material = geometry->material;
  vec3f Ng = ray.Ng;
  vec3f Ns = Ng;

  if (flags & DG_NORMALIZE) {
    Ng = normalize(Ng);
    Ns = normalize(Ns);
  }
  if (flags & DG_FACEFORWARD) {
    if (dot(ray.dir,Ng) >= 0.f) Ng = neg(Ng);
    if (dot(ray.dir,Ns) >= 0.f) Ns = neg(Ns);
  }

  if ((flags & DG_COLOR) && (THIS->attribute != NULL) && (THIS->transferFunction != NULL)) {
    // normalize attribute to the [0,1] range (by normalizing relative
    // to the attribute range stored in the min max BVH's root node
    const uniform float attrib_lo = THIS->attr_lo;
    const uniform float attrib_hi = THIS->attr_hi;
    const float attrib
      = (THIS->attribute[ray.primID] - attrib_lo)
      * rcp(attrib_hi - attrib_lo + 1e-10f);
    
    // compute alpha value from attribute value
    const float alpha = THIS->transferFunction->getOpacityForValue(THIS->transferFunction,
                                                                   attrib);
    const vec3f color = THIS->transferFunction->getColorForValue(THIS->transferFunction,
                                                                 attrib);
    dg.color = make_vec4f(color.x,color.y,color.z,alpha);
  }
  dg.Ng = Ng;
  dg.Ns = Ns;
}

void PartiKDGeometry_bounds(uniform PartiKDGeometry *uniform geometry,
                            uniform size_t primID,
                            uniform box3fa &bbox)
{
  bbox.lower = geometry->sphereBounds.lower;
  bbox.upper = geometry->sphereBounds.upper;
}


/*! creates a new pkd geometry */
export void *uniform PartiKDGeometry_create(void           *uniform cppEquivalent)
{
  uniform PartiKDGeometry *uniform geom = uniform new uniform PartiKDGeometry;
  Geometry_Constructor(&geom->geometry,cppEquivalent,
                       PartiKDGeometry_postIntersect,
                       NULL,0,NULL);
  return geom;
}

/*! helper function that recomputes the bin-bits from the transfer
    function for parameter range culling. note this function does
    _NOT_ change anything in the tree itself, it only re-bins the
    transfer function*/
export void PartiKDGeometry_updateTransferFunction(void *uniform _THIS,
                                                   void *uniform _transferFunction)
{
  PartiKDGeometry *uniform THIS = (PartiKDGeometry *uniform)_THIS;
  TransferFunction *uniform transferFunction
    = (TransferFunction *uniform)_transferFunction;
  THIS->transferFunction_activeBinBits = 0;
  for (uniform int i=0;i<32;i++) {
    uniform float a0 = i/32.f;
    uniform float a1 = (i+1)/32.f - 1e-5f;
    vec2f range = make_vec2f(a0,a1);
    uniform float alphaRange
      = extract(transferFunction->getMaxOpacityInRange(transferFunction,range),0);
    if (alphaRange >= .5f)
      THIS->transferFunction_activeBinBits |= (1UL << i);
  }
}

/*! 'constructor' for a newly created pkd geometry */
export void PartiKDGeometry_set(void       *uniform _geom,
                                void           *uniform _model,
                                uniform bool useSPMD,
                                void           *uniform transferFunction,
                                float           uniform particleRadius,
                                uniform size_t  numParticles,
                                uniform size_t  numInnerNodes,
                                PKDParticle    *uniform particle,
                                float          *uniform attribute,
                                uint32         *uniform innerNode_attributeMask,
                                uniform box3f &centerBounds,
                                uniform box3f &sphereBounds,
                                uniform float attr_lo, 
                                uniform float attr_hi)
{
  uniform PartiKDGeometry *uniform geom = (uniform PartiKDGeometry *uniform)_geom;
  uniform Model *uniform model = (uniform Model *uniform)_model;

  uniform uint32 geomID = rtcNewUserGeometry(model->embreeSceneHandle,1);
  
  geom->geometry.model  = model;
  geom->geometry.geomID = geomID;
  geom->particleRadius  = particleRadius;
  geom->particle        = particle;
  geom->numParticles    = numParticles;
  geom->numInnerNodes   = numInnerNodes;
  geom->centerBounds    = centerBounds;
  geom->sphereBounds    = sphereBounds;
  geom->attribute       = attribute;
  geom->attr_lo         = attr_lo;
  geom->attr_hi         = attr_hi;
  geom->innerNode_attributeMask   = innerNode_attributeMask;
  print("array of inner-node attribute masks: %\n",innerNode_attributeMask);

  geom->transferFunction  = (TransferFunction *uniform)transferFunction;

  rtcSetUserData(model->embreeSceneHandle,geomID,geom);
  rtcSetBoundsFunction(model->embreeSceneHandle,geomID,
                       (uniform RTCBoundsFunc)&PartiKDGeometry_bounds);

  if (useSPMD) {
  print("creating PKD with ***SPMD*** traversal\n");
  rtcSetIntersectFunction
    (model->embreeSceneHandle,geomID,
     (uniform RTCIntersectFuncVarying)&PartiKDGeometry_intersect_spmd);
  rtcSetOccludedFunction
    (model->embreeSceneHandle,geomID,
     (uniform RTCOccludedFuncVarying)&PartiKDGeometry_occluded_spmd);
  } else {
  rtcSetIntersectFunction
    (model->embreeSceneHandle,geomID,
     (uniform RTCIntersectFuncVarying)&PartiKDGeometry_intersect_packet);
  rtcSetOccludedFunction
    (model->embreeSceneHandle,geomID,
     (uniform RTCOccludedFuncVarying)&PartiKDGeometry_occluded_packet);
  }
  if (transferFunction) 
    PartiKDGeometry_updateTransferFunction(geom,transferFunction);
}
