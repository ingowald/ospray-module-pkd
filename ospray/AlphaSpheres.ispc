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
#include "ospray/math/box.ih"
#include "ospray/common/Ray.ih"
#include "ospray/geometry/Geometry.ih"
#include "ospray/common/Model.ih"
#include "ospray/transferFunction/LinearTransferFunction.ih"
// this module
#include "MinMaxBVH2.ih"
// embree
#include "embree2/rtcore.isph"
#include "embree2/rtcore_scene.isph"
#include "embree2/rtcore_geometry_user.isph"

#define USE_NAIVE_SPMD_TRAVERSAL 0

struct AlphaSpheres {
  uniform Geometry geometry; //!< inherited geometry fields

  vec3f *uniform position;
  float *uniform attribute;

  //  uniform Material *uniform *materialList;
  uniform TransferFunction *uniform transferFunction;

  // the min-max BVH over the primitives
  MinMaxBVH2      mmBVH;

  float           radius;
  int32           numSpheres;
};

typedef uniform float uniform_float;

static void AlphaSpheres_postIntersect(uniform Geometry *uniform geometry,
                                       uniform Model *uniform model,
                                       varying DifferentialGeometry &dg,
                                       const varying Ray &ray,
                                       uniform int64 flags)
{
  uniform AlphaSpheres *uniform self = (uniform AlphaSpheres *uniform)geometry;
  dg.Ng = dg.Ns = ray.Ng;

  if ((flags & DG_COLOR) && (self->attribute) && (self->transferFunction)) {
    // compute base pointer to actual sphere we are intersecting
    foreach_unique(primID in ray.primID) {
      uniform float attrib = self->attribute[primID];

      // normalize attribute to the [0,1] range (by normalizing relative
      // to the attribute range stored in the min max BVH's root node
      const uniform float attrib_lo = self->mmBVH.node[0].range_lo;
      const uniform float attrib_hi = self->mmBVH.node[0].range_hi;
      attrib = (attrib - attrib_lo) * rcp(attrib_hi - attrib_lo + 1e-10f);

      // compute alpha value from attribute value
      float alpha = self->transferFunction->getOpacityForValue(self->transferFunction,
                                                               attrib);
      vec3f color = self->transferFunction->getColorForValue(self->transferFunction,
                                                             attrib);
      dg.color = make_vec4f(color.x,color.y,color.z,alpha);
    }
  }
}

void AlphaSpheres_bounds(uniform AlphaSpheres *uniform geometry,
                         uniform size_t primID,
                         uniform box3fa &bbox)
{
  bbox = make_box3fa(geometry->mmBVH.node[0].bounds_lo,
                     geometry->mmBVH.node[0].bounds_hi);
}

static
varying bool AlphaSpheres_intersectPrim(void *uniform geomPtr,
                                        uniform int primID,
                                        varying Ray &ray)
{
  // typecast "implicit self" pointer to the proper geometry type
  AlphaSpheres *uniform self = (AlphaSpheres *uniform)geomPtr;

  // read sphere members required for intersection test
  uniform float radius = self->radius;
  uniform vec3f center = self->position[primID];

  // perform first half of intersection test ....
  const vec3f A = center - ray.org;

  const float a = dot(ray.dir,ray.dir);
  const float b = -2.f*dot(ray.dir,A);
  const float c = dot(A,A)-radius*radius;
  
  const float radical = b*b-4.f*a*c;
  if (radical < 0.f) return false;


  // compute second half of intersection test
  const float srad = sqrt(radical);

  const float t_in = (- b - srad) *rcpf(2.f*a);
  const float t_out= (- b + srad) *rcpf(2.f*a);

  float hit_t = 0.f;
  if (t_in > ray.t0 && t_in < ray.t) {
    hit_t = t_in;
  } else if (t_out > ray.t0 && t_out < ray.t) {
    hit_t = t_out;
  }
  else /* miss : */ return false;

  if (self->attribute && self->transferFunction) {
    // -------------------------------------------------------
    // do attribute test
    uniform float attrib = self->attribute[primID];

    // normalize attribute to the [0,1] range (by normalizing relative
    // to the attribute range stored in the min max BVH's root node
    const uniform float attrib_lo = self->mmBVH.node[0].range_lo;
    const uniform float attrib_hi = self->mmBVH.node[0].range_hi;
    attrib = (attrib - attrib_lo) * rcp(attrib_hi - attrib_lo + 1e-10f);

    // compute alpha value from attribute value
    const float alpha
      = self->transferFunction->getOpacityForValue(self->transferFunction,attrib);
    if (alpha <= .5f)
      return false;
  }

  // found a hit - store it
  ray.primID = primID;
  ray.geomID = self->geometry.geomID;
  ray.t = t_in;
  ray.Ng = ray.org + ray.t*ray.dir - center;
  return true;
}

void AlphaSpheres_intersect(uniform AlphaSpheres *uniform geometry,
                            varying Ray &ray,
                            uniform size_t primID)
{
  MinMaxBVH2_intersect_packet(&geometry->mmBVH,geometry,geometry->transferFunction,
                              &AlphaSpheres_intersectPrim,geometry->numSpheres,ray);
}

void AlphaSpheres_occluded(uniform AlphaSpheres *uniform geometry,
                           varying Ray &ray,
                           uniform size_t primID)
{
  MinMaxBVH2_occluded_packet(&geometry->mmBVH,geometry,geometry->transferFunction,
                             &AlphaSpheres_intersectPrim,geometry->numSpheres,ray);
}


export void *uniform AlphaSpheres_create(void           *uniform cppEquivalent)
{
  uniform AlphaSpheres *uniform geom = uniform new uniform AlphaSpheres;
  Geometry_Constructor(&geom->geometry,cppEquivalent,
                       AlphaSpheres_postIntersect,
                       NULL,0,NULL);
  return geom;
}


export void AlphaSpheres_set(void           *uniform _geom,
                             void           *uniform _model,
                             void           *uniform transferFunction,
                             int64           uniform rootRef,
                             const void     *uniform bvhNode,
                             const uint32   *uniform primID,
                             float           uniform radius,
                             vec3f          *uniform positionData,
                             float          *uniform attributeData,
                             int             uniform numSpheres)
{
  uniform AlphaSpheres *uniform geom = (uniform AlphaSpheres *uniform)_geom;
  uniform Model *uniform model = (uniform Model *uniform)_model;

  uniform uint32 geomID = rtcNewUserGeometry(model->embreeSceneHandle,1); //numSpheres);
  
  geom->geometry.model = model;
  geom->geometry.geomID = geomID;
  // geom->materialList = (Material **)materialList;
  geom->numSpheres = numSpheres;
  geom->radius = radius;
  geom->position  = positionData;
  geom->attribute = attributeData;

  print("radius %\n",radius);

  geom->mmBVH.rootRef = rootRef;
  geom->mmBVH.node    = (MinMaxBVH2Node*uniform)bvhNode;
  geom->mmBVH.primID  = primID;

  geom->transferFunction  = (TransferFunction *uniform)transferFunction;

  rtcSetUserData(model->embreeSceneHandle,geomID,geom);
  rtcSetBoundsFunction(model->embreeSceneHandle,geomID,
                       (uniform RTCBoundsFunc)&AlphaSpheres_bounds);
  rtcSetIntersectFunction(model->embreeSceneHandle,geomID,
                          (uniform RTCIntersectFuncVarying)&AlphaSpheres_intersect);
  rtcSetOccludedFunction(model->embreeSceneHandle,geomID,
                         (uniform RTCOccludedFuncVarying)&AlphaSpheres_occluded);
}





// export void PKDGeometry_set(void           *uniform _geom,
//                             void           *uniform _model,
//                             void           *uniform transferFunction,
//                             float           uniform radius,
//                             PKDParticle *uniform particle,
//                             uniform size_t numParticles,
//                             uint32 *uniform innerNodeInfo,
//                             uniform size_t numInnerNodes,
//                             uniform box3f &centerBounds,
//                             uniform box3f &sphereBounds,
//                             uniform float attr_lo, uniform float attr_hi)
// {
//   uniform PKDGeometry *uniform geom = (uniform PKDGeometry *uniform)_geom;
//   uniform Model *uniform model = (uniform Model *uniform)_model;

//   uniform uint32 geomID = rtcNewUserGeometry(model->embreeSceneHandle,1); //numSpheres);
  
//   geom->geometry.model = model;
//   geom->geometry.geomID = geomID;
//   geom->radius = radius;
//   geom->particle  = particle;
//   geom->numParticles  = numParticles;
//   geom->innerNodeInfo  = innerNodeInfo;
//   geom->numInnerNodes = numInnerNodes;
//   geom->centerBounds        = centerBounds;
//   geom->sphereBounds        = sphereBounds;
//   geom->attr_lo = attr_lo;
//   geom->attr_hi = attr_hi;

//   geom->activeBinBits = 0xffffffff;

//   geom->transferFunction  = (TransferFunction *uniform)transferFunction;

//   rtcSetUserData(model->embreeSceneHandle,geomID,geom);
//   rtcSetBoundsFunction(model->embreeSceneHandle,geomID,
//                        (uniform RTCBoundsFunc)&PKDGeometry_bounds);
// #if USE_NAIVE_SPMD_TRAVERSAL
//   rtcSetIntersectFunction(model->embreeSceneHandle,geomID,
//                           (uniform RTCIntersectFuncVarying)&PKDGeometry_intersect_naive);
//   rtcSetOccludedFunction(model->embreeSceneHandle,geomID,
//                          (uniform RTCOccludedFuncVarying)&PKDGeometry_occluded_naive);
// #else
//   rtcSetIntersectFunction(model->embreeSceneHandle,geomID,
//                           (uniform RTCIntersectFuncVarying)&PKDGeometry_intersect_packet);
//   rtcSetOccludedFunction(model->embreeSceneHandle,geomID,
//                          (uniform RTCOccludedFuncVarying)&PKDGeometry_occluded_packet);
// #endif
// }
