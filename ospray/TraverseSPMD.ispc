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
#include "PKDGeometry.ih"
// embree
#include "embree2/rtcore.isph"
#include "embree2/rtcore_scene.isph"
#include "embree2/rtcore_geometry_user.isph"

#define modify_radius(t) 4.f

struct ThreePhaseStackEntry {
  float t_in, t_out, t_sphere_out;
  size_t sphereID;
  size_t farChildID;
#if DIM_FROM_DEPTH
  int32  dim;
#endif
};

inline varying bool PartiKDGeometry_intersectPrim(void *uniform geomPtr,
                                                  varying size_t primID,
                                                  varying Ray &ray)
{
  // typecast "implicit self" pointer to the proper geometry type
  PartiKDGeometry *uniform self = (PartiKDGeometry *uniform)geomPtr;
  const uniform float *varying pos = &self->particle[primID].position[0];
  // read sphere members required for intersection test
  const float radius = self->particleRadius * modify_radius(ray.t);
  // uniform vec3f center = (uniform vec3f &)self->particle[primID].position;
  const vec3f center = make_vec3f((varying float)pos[0],pos[1],pos[2]);
  
  // perform first half of intersection test ....
  const vec3f A = center - ray.org;

  const float a = dot(ray.dir,ray.dir);
  const float b = -2.f*dot(ray.dir,A);
  const float c = dot(A,A)-radius*radius;
  
  const float radical = b*b-4.f*a*c;
  if (radical < 0.f) return false;

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
  if ((self->attribute!=NULL) & (self->transferFunction!=NULL)) {
    // -------------------------------------------------------
    // do attribute test
    float attrib = self->attribute[primID];

    // normalize attribute to the [0,1] range (by normalizing relative
    // to the attribute range stored in the min max BVH's root node
    const uniform float attrib_lo = self->attr_lo;
    const uniform float attrib_hi = self->attr_hi;
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
  ray.Ng = ray.t*ray.dir - A;
  return true;
}

inline void pkd_traverse_spmd(uniform PartiKDGeometry *uniform self,
                                   varying Ray &ray,
                                   const varying float rdir[3], 
                                   const varying float org[3],
                                   const varying float t_in_0, 
                                   const varying float t_out_0,
                                   const varying size_t dir_sign[3],
                                   const uniform bool isShadowRay
                                   )
{
  varying ThreePhaseStackEntry stack[32];
  varying ThreePhaseStackEntry *varying stackPtr = stack;
  
  size_t nodeID = 0;
  size_t dim    = 0;
  
  float t_in = t_in_0;
  float t_out = t_out_0;
  const float radius = self->particleRadius * modify_radius(t_in_0);
  const uniform size_t numInnerNodes = self->numInnerNodes;
  const uniform size_t numParticles  = self->numParticles;
  const uniform PKDParticle *uniform const particle = self->particle;
  while (1) {
    // ------------------------------------------------------------------
    // do traversal step(s) as long as possible
    // ------------------------------------------------------------------
    while (1) {    

      if (t_in >= t_out) break;
      
      if (nodeID >= numInnerNodes) {
        // this is a leaf node - can't to to a leaf, anyway. Intersect
        // the prim, and be done with it.
        PartiKDGeometry_intersectPrim(self,nodeID,ray);
        if (isShadowRay && ray.primID >= 0) return;
        break;
      } 


      if (self->innerNode_attributeMask) {
        const uint32 nodeAttrBits = self->innerNode_attributeMask[nodeID];
        if ((nodeAttrBits & self->transferFunction_activeBinBits) == 0)
          break;
      }

#if !DIM_FROM_DEPTH
      INT3 *uniform intPtr = (INT3 *uniform)self->particle;
      dim = intPtr[nodeID].x & 3;
#endif

      const  size_t sign = dir_sign[dim];
      
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

      // case where ray segment is FULLY on far side (including far of shphere)
      if (t_in >= t_nearChild_out) {
        t_in  = t_farChild_in;
        //t_out = t_farChild_out;
        nodeID = 2*nodeID+2-sign;
        continue;
      }

      // case where ray segment is FULLY on near side (including near of sphere)
      if (t_out <= t_farChild_in) {
        //        t_in  = t_nearChild_in;
        t_out = t_nearChild_out;
        nodeID = 2*nodeID+1+sign;
        continue;
      }

      // else, we're on both sides:
#if DIM_FROM_DEPTH
      dim = (dim == 2)?0:dim+1;
      stackPtr->dim        = dim;
#endif
      stackPtr->farChildID   = 2*nodeID+2-sign;
      stackPtr->t_in         = t_farChild_in;
      stackPtr->t_out        = t_farChild_out;
      stackPtr->t_sphere_out = t_nearChild_out;

      //t_in  = t_nearChild_in;
      t_out = t_nearChild_out;
      nodeID = min(2*nodeID+1+sign,numParticles-1);
      stackPtr->sphereID   = nodeID;
      
      ++stackPtr;

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
        PartiKDGeometry_intersectPrim(self,stackPtr->sphereID,ray);
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
inline void pkd_traverse_spmd(uniform PartiKDGeometry *uniform self,
                              varying Ray &ray,
                              uniform size_t primID,
                              uniform bool isShadowRay)
{
  float t_in = ray.t0, t_out = ray.t;
  intersectBox(ray,self->sphereBounds,t_in,t_out);

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

  size_t dir_sign[3];
  dir_sign[0] = ray.dir.x < 0.f;
  dir_sign[1] = ray.dir.y < 0.f;
  dir_sign[2] = ray.dir.z < 0.f;

  pkd_traverse_spmd(self,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
}

/*! the 'virtual' traverse function for a pkd geometry */
void PartiKDGeometry_intersect_spmd(uniform PartiKDGeometry *uniform self,
                                      varying Ray &ray,
                                      uniform size_t primID)
{ pkd_traverse_spmd(self,ray,primID,false); }

/*! the 'virtual' occluded function for a pkd geometry */
void PartiKDGeometry_occluded_spmd(uniform PartiKDGeometry *uniform self,
                                     varying Ray &ray,
                                     uniform size_t primID)
{ pkd_traverse_spmd(self,ray,primID,true); }


