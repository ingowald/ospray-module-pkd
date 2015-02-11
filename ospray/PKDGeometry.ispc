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


//#define SPLATTING_TEST 1


/*! if enabled, we use a naive SPMD traversal where each lane does its own traversal */
// #define USE_NAIVE_SPMD_TRAVERSAL 0

/*! if defined, we'll use 'depth%3' for the partition dim; if not,
  we'll use the lower two mantissa bits of the particle's x
  coordinate for the split plane dimesnsion (this assumes that the
  pkd tree has been built as such */
//#define DIM_FROM_DEPTH 1

inline float safe_rcp(float f) 
{ return (abs(f) < 1e-20f)?1e20f:rcp(f); }

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

#if SPLATTING_TEST
  if (flags & DG_COLOR) {
    dg.color = make_vec4f(ray.Ng.x,0.f,0.f,0.f);
    dg.Ns = dg.Ng = normalize(neg(ray.dir));
  }
#else  
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
#endif
}

void PartiKDGeometry_bounds(uniform PartiKDGeometry *uniform geometry,
                            uniform size_t primID,
                            uniform box3fa &bbox)
{
  bbox.lower = geometry->sphereBounds.lower;
  bbox.upper = geometry->sphereBounds.upper;
}


inline
varying bool PartiKDGeometry_intersectPrim(void *uniform geomPtr,
                                           uniform size_t primID,
                                           varying Ray &ray)
{
  // typecast "implicit THIS" pointer to the proper geometry type
  PartiKDGeometry *uniform THIS = (PartiKDGeometry *uniform)geomPtr;
  const uniform float *uniform pos = &THIS->particle[primID].position[0];
  // read sphere members required for intersection test
  const float radius = THIS->particleRadius;
  // uniform vec3f center = (uniform vec3f &)THIS->particle[primID].position;
  const vec3f center = make_vec3f((varying float)pos[0],pos[1],pos[2]);
  
  // perform first half of intersection test ....
  const vec3f A = center - ray.org;

  const float a = dot(ray.dir,ray.dir);
  const float b = -2.f*dot(ray.dir,A);
  const float c = dot(A,A)-radius*radius;
  
  const float radical = b*b-4.f*a*c;
  if (radical < 0.f) return false;

#if SPLATTING_TEST
  vec3f splatColor = make_vec3f(1e-3f); //*(srad)*rcpf(a+a));
  ray.primID = 0;
  ray.geomID = THIS->geometry.geomID;
  ray.Ng = ray.Ng + splatColor;
  ray.t = 1e5f;
#else

  // compute second half of intersection test
  const float srad = sqrt(radical);
  
  const float t_in  = (- b - srad) *rcpf(a+a);
  const float t_out = (- b + srad) *rcpf(a+a);

  float hit_t = 0.f;
  if (t_in > ray.t0 && t_in < ray.t) {
    hit_t = t_in;
  } else if (t_out > ray.t0 && t_out < ray.t) {
    hit_t = t_out;
  }
  else /* miss : */ return false;

  // do attribute alpha test, if both attribute and transfer fct are set
  if ((THIS->attribute!=NULL) & (THIS->transferFunction!=NULL)) {
    // -------------------------------------------------------
    // do attribute test
    uniform float attrib = THIS->attribute[primID];

    // normalize attribute to the [0,1] range (by normalizing relative
    // to the attribute range stored in the min max BVH's root node
    const uniform float attrib_lo = THIS->attr_lo;
    const uniform float attrib_hi = THIS->attr_hi;
    attrib = (attrib - attrib_lo) * rcp(attrib_hi - attrib_lo + 1e-10f);
  
    // compute alpha value from attribute value
    const float alpha
      = THIS->transferFunction->getOpacityForValue(THIS->transferFunction,attrib);
    if (alpha <= .5f)
      return false;
  }

  // found a hit - store it
  ray.primID = primID;
  ray.geomID = THIS->geometry.geomID;
  ray.t = t_in;
  ray.Ng = ray.t*ray.dir - A;
#endif
  return true;
}

#if DIM_FROM_DEPTH
# define IF_HAS_DEPTH(a) a
#else
# define IF_HAS_DEPTH(a) 
#endif

struct PartiKDStackEntry {
  varying float t_in, t_out;
  uniform size_t nodeID;
};






struct ThreePhaseStackEntry {
  varying float t_in, t_out, t_sphere_out;
  uniform size_t sphereID;
  uniform size_t farChildID;
#if DIM_FROM_DEPTH
  uniform int32  dim;
#endif
};


inline void pkd_traverse_packet(uniform PartiKDGeometry *uniform THIS,
                                   varying Ray &ray,
                                   const varying float rdir[3], 
                                   const varying float org[3],
                                   const varying float t_in_0, 
                                   const varying float t_out_0,
                                   const uniform size_t dir_sign[3],
                                   const uniform bool isShadowRay
                                   )
{
  varying ThreePhaseStackEntry stack[32];
  varying ThreePhaseStackEntry *uniform stackPtr = stack;
  
  uniform size_t nodeID = 0;
  uniform size_t dim    = 0;
  
  float t_in = t_in_0;
  float t_out = t_out_0;
  const float radius = THIS->particleRadius;
  const uniform size_t numInnerNodes = THIS->numInnerNodes;
  const uniform size_t numParticles  = THIS->numParticles;
  const uniform PKDParticle *uniform const particle = THIS->particle;
  while (1) {
    // ------------------------------------------------------------------
    // do traversal step(s) as long as possible
    // ------------------------------------------------------------------
    while (1) {    

      if (t_in > t_out) break;

      if (nodeID >= numInnerNodes) {
        // this is a leaf node - can't to to a leaf, anyway. Intersect
        // the prim, and be done with it.
        PartiKDGeometry_intersectPrim(THIS,nodeID,ray);
        if (isShadowRay && ray.primID >= 0) return;
        break;
      } 


      if (THIS->innerNode_attributeMask) {
        const uniform uint32 nodeAttrBits = THIS->innerNode_attributeMask[nodeID];
        if ((nodeAttrBits & THIS->transferFunction_activeBinBits) == 0)
          break;
      }

#if !DIM_FROM_DEPTH
      INT3 *uniform intPtr = (INT3 *uniform)THIS->particle;
      dim = intPtr[nodeID].x & 3;
#endif

      const uniform size_t sign = dir_sign[dim];
      
      // ------------------------------------------------------------------
      // traversal step: compute distance, then compute intervals for front and back side
      // ------------------------------------------------------------------
      const float org_to_node_dim = particle[nodeID].position[dim] - org[dim];
      const float t_plane_0  = (org_to_node_dim - radius) * rdir[dim];
      const float t_plane_1  = (org_to_node_dim + radius) * rdir[dim];
      const float t_plane_nr = min(t_plane_0,t_plane_1);
      const float t_plane_fr = max(t_plane_0,t_plane_1);

      const float t_farChild_in   = max(t_in,t_plane_nr);
      const float t_farChild_out  = t_out;
      const float t_nearChild_out = min(t_out,t_plane_fr);

      // catch the case where all ray segments are on far side
      if (none(t_in < t_nearChild_out)) {
        if (none(t_farChild_in < t_farChild_out)) {
          break;
        } else {
          t_in  = t_farChild_in;
          t_out = t_farChild_out;
          nodeID = 2*nodeID+2-sign;
          continue;
        }
      }

      unmasked { 
        stackPtr->t_in = 1e20f;
        stackPtr->t_out = -1e20f;
        stackPtr->t_sphere_out = -1e20f;
      }
      stackPtr->farChildID = 2*nodeID+2-sign;
      
      stackPtr->t_in       = t_farChild_in;
      stackPtr->t_out      = t_farChild_out;
      stackPtr->t_sphere_out = t_nearChild_out;

      t_out = t_nearChild_out; 
#if DIM_FROM_DEPTH
      dim = (dim == 2)?0:dim+1;
      
      stackPtr->dim        = dim;
#endif
      stackPtr->sphereID   = nodeID;

      if (any(t_farChild_in < t_farChild_out)) 
        ++stackPtr;
      
      if (none(t_in < t_out)) 
        break;

      nodeID = min(2*nodeID+1+sign,numParticles-1);

      continue;
    }
    // ------------------------------------------------------------------
    // couldn't go down any further; pop a node from stack
    // ------------------------------------------------------------------
    while (1) {
      // pop as long as we have to ... or until nothing is left to pop.
      if (stackPtr == stack) {
        return;
      }
      unmasked { 
        t_in   = stackPtr[-1].t_in;
        t_out  = min(stackPtr[-1].t_out,ray.t);
      }
      -- stackPtr;

      // check if the node is still active (all the traversal since it
      // originally got pushed may have shortened the ray)
      if (none(t_in < t_out))
        continue;

      // intersect the actual node...
      if (t_in < min(stackPtr->t_sphere_out,ray.t)) {
        PartiKDGeometry_intersectPrim(THIS,stackPtr->sphereID,ray);
        if (isShadowRay && ray.primID >= 0) return;
      } 
      
      // do the distance test again, we might just have shortened the ray...
      unmasked { t_out  = min(t_out,ray.t); }
      nodeID = min(stackPtr->farChildID,numParticles-1);
#if DIM_FROM_DEPTH
      dim    = stackPtr->dim;
#endif
      break;
    }
  }
}

/*! generic traverse/occluded function that splits the packet into
    subpackets of equal sige, and then calls the appropiate
    constant-sign traverse function. this method works for both shadow
    and primary rays, as indicated by the 'isShadowRay' flag */
inline void pkd_traverse_packet(uniform PartiKDGeometry *uniform THIS,
                               varying Ray &ray,
                               uniform size_t primID,
                               uniform bool isShadowRay)
{
  float t_in = ray.t0, t_out = ray.t;
  boxtest(ray,THIS->sphereBounds,t_in,t_out);

  if (t_out < t_in)
    return;
  
  const varying float rdir[3] = { 
    safe_rcp(ray.dir.x),
    safe_rcp(ray.dir.y),
    safe_rcp(ray.dir.z) 
  };
  const varying float org[3]  = { 
    ray.org.x, 
    ray.org.y, 
    ray.org.z 
  };

  uniform size_t dir_sign[3];
  if (ray.dir.z > 0.f) {
    dir_sign[2] = 0;
    if (ray.dir.y > 0.f) {
      dir_sign[1] = 0;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_traverse_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      }
    } else {
      dir_sign[1] = 1;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_traverse_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      }
    }
  } else {
    dir_sign[2] = 1;
    if (ray.dir.y > 0.f) {
      dir_sign[1] = 0;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_traverse_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      }
    } else {
      dir_sign[1] = 1;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_traverse_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      }
    }
  }
}

/*! the 'virtual' traverse function for a pkd geometry */
void PartiKDGeometry_intersect_packet(uniform PartiKDGeometry *uniform THIS,
                                      varying Ray &ray,
                                      uniform size_t primID)
{ pkd_traverse_packet(THIS,ray,primID,false); }

/*! the 'virtual' occluded function for a pkd geometry */
void PartiKDGeometry_occluded_packet(uniform PartiKDGeometry *uniform THIS,
                                     varying Ray &ray,
                                     uniform size_t primID)
{ pkd_traverse_packet(THIS,ray,primID,true); }


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
  
  rtcSetIntersectFunction
    (model->embreeSceneHandle,geomID,
     (uniform RTCIntersectFuncVarying)&PartiKDGeometry_intersect_packet);
  rtcSetOccludedFunction
    (model->embreeSceneHandle,geomID,
     (uniform RTCOccludedFuncVarying)&PartiKDGeometry_occluded_packet);
  
  if (transferFunction) 
    PartiKDGeometry_updateTransferFunction(geom,transferFunction);
}
