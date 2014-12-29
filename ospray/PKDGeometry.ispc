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
#include "ospray/geometry/geometry.ih"
#include "ospray/common/Model.ih"
#include "ospray/transferfunction/LinearTransferFunction.ih"
// this module
#include "PKDGeometry.ih"
// embree
#include "embree2/rtcore.isph"
#include "embree2/rtcore_scene.isph"
#include "embree2/rtcore_geometry_user.isph"

#include "ospray/common/Box.ih"

/*! if enabled, we use a naive SPMD traversal where each lane does its own traversal */
#define USE_NAIVE_SPMD_TRAVERSAL 0

/*! if defined, we'll use 'depth%3' for the partition dim; if not,
  we'll use the lower two mantissa bits of the particle's x
  coordinate for the split plane dimesnsion (this assumes that the
  pkd tree has been built as such */
#define DIM_FROM_DEPTH 1


// uniform int64 dbg_numIntersections = 0;
// uniform int64 dbg_numRays = 0;

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
    // float attrib = ;//particle[primID].attribute;
    
    // normalize attribute to the [0,1] range (by normalizing relative
    // to the attribute range stored in the min max BVH's root node
    const uniform float attrib_lo = THIS->attr_lo;
    const uniform float attrib_hi = THIS->attr_hi;
    const float attrib = (THIS->attribute[ray.primID] - attrib_lo) * rcp(attrib_hi - attrib_lo + 1e-10f);
    
    // compute alpha value from attribute value
    const float alpha = THIS->transferFunction->getOpacityForValue(THIS->transferFunction,
                                                                   attrib);
    const vec3f color = THIS->transferFunction->getColorForValue(THIS->transferFunction,
                                                                 attrib);
    dg.color = make_vec4f(color.x,color.y,color.z,alpha);
    // }
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
  // print("bounds.lower % % %\n",bbox.lower.x,bbox.lower.y,bbox.lower.z);
  // print("bounds.upper % % %\n",bbox.upper.x,bbox.upper.y,bbox.upper.z);
}


inline
varying bool PartiKDGeometry_intersectPrim(void *uniform geomPtr,
                                           uniform size_t primID,
                                           varying Ray &ray)
{
  // dbg_numIntersections++;
  //  print("isecprim\n");
  // typecast "implicit THIS" pointer to the proper geometry type
  PartiKDGeometry *uniform THIS = (PartiKDGeometry *uniform)geomPtr;
  const uniform float *uniform pos = &THIS->particle[primID].position[0];
  // read sphere members required for intersection test
  const float radius = THIS->particleRadius;
  // uniform vec3f center = (uniform vec3f &)THIS->particle[primID].position;
  const vec3f center = make_vec3f((varying float)pos[0],pos[1],pos[2]);
// THIS->particle[primID].position[0],
//                                   THIS->particle[primID].position[1],
//                                   THIS->particle[primID].position[2]);
  
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
  // ray.Ng = ray.org + ray.t*ray.dir - center;
  return true;
}


#if USE_NAIVE_SPMD_TRAVERSAL 
#else

#if DIM_FROM_DEPTH
# define IF_HAS_DEPTH(a) a
#else
# define IF_HAS_DEPTH(a) 
#endif


struct ThreePhaseStackEntry {
  varying float t_in, t_out;
  uniform size_t sphereID;
  uniform size_t farChildID;
  uniform int32  dim;
};



inline void PartiKDGeometry_packet(uniform PartiKDGeometry *uniform THIS,
                                   varying Ray &ray,
                                   const varying float rdir[3], const varying float org[3],
                                   const varying float t_in_0, const varying float t_out_0,
                                   const uniform size_t dir_sign[3],
                                   const uniform bool isShadowRay
                                   )
{
  varying ThreePhaseStackEntry stack[64];
  varying ThreePhaseStackEntry *uniform stackPtr = stack;
  
  uniform size_t nodeID = 0;
  uniform size_t dim    = 0;
  
  float t_in = t_in_0;
  float t_out = t_out_0;
  const float radius = THIS->particleRadius;
  const uniform size_t numInnerNodes = THIS->numInnerNodes;
  const uniform size_t numParticles  = THIS->numParticles;
  const uniform PKDParticle *uniform const particle = THIS->particle;
  // print("new traversal % %\n",t_in,t_out);
  while (1) {
    // ------------------------------------------------------------------
    // do traversal step(s) as long as possible
    // ------------------------------------------------------------------
    // print("outer % % %\n",nodeID,t_in,t_out);
    while (1) {    
      // print("inner node % dim % range % %\n",nodeID,dim,t_in,t_out);
      if (nodeID >= numInnerNodes) {
        // print("LEAF prim %\n",nodeID);
        // this is a leaf node - can't to to a leaf, anyway. Intersect the prim, and be done with it.
        PartiKDGeometry_intersectPrim(THIS,nodeID,ray);
        if (isShadowRay && ray.primID >= 0) return;
        break;
      } 
      
      const uniform size_t sign = dir_sign[dim];
      // print("sign %\n",sign);
      
      // ------------------------------------------------------------------
      // traversal step: compute distance, then compute intervals for front and back side
      // ------------------------------------------------------------------
      const float org_to_node_dim = particle[nodeID].position[dim] - org[dim];
      const float t_plane_0  = (org_to_node_dim - radius) * rdir[dim];
      const float t_plane_1  = (org_to_node_dim + radius) * rdir[dim];
      const float t_plane_nr = min(t_plane_0,t_plane_1);
      const float t_plane_fr = max(t_plane_0,t_plane_1);

      // print("t_plane % %\n",t_plane_nr,t_plane_fr);

      unmasked { 
        const float t_farChild_in   = max(t_in,t_plane_nr);
        const float t_farChild_out  = t_out;
      
        stackPtr->t_in       = t_farChild_in;
        stackPtr->t_out      = t_farChild_out;
      }

      // const float t_nearChild_in  = t_in;
      // const float t_nearChild_out = min(t_out,t_plane_fr);
      unmasked { t_out = min(t_out,t_plane_fr); }
      dim = (dim == 2)?0:dim+1;
      // const uniform size_t nextDim = (dim == 2)?0:dim+1;
      
      unmasked {
        stackPtr->dim        = dim; //nextDim;
        stackPtr->sphereID   = nodeID;
        stackPtr->farChildID = 2*nodeID+2-sign;
      }
      if (any(t_farChild_in <= t_farChild_out)) {
        // could actually do those writes, anyway (even when not
        // actuaqlly going into this subtree), thus interleaving
        // them with the computations above.
        ++stackPtr;
      }
      
#if 0
      nodeID = min(2*nodeID+1+sign,numParticles-1);
      if (all(t_nearChild_in >= t_nearChild_out)) break;
#else
      nodeID = 2*nodeID+1+sign;
      if (// cannot go down greedily, go to popping.
          all(t_in >= t_out)
          // all(t_nearChild_in >= t_nearChild_out)
          ||
          // check if NEAR child exists. so far (from the tests above)
          // we only know that SOME child exists, and that we should go
          // to the near one if it does exist.... but we might still be
          // in the corner case that we only have a single child, and it
          // is the far one.... so let's check that here. (OPTIMIZATION:
          // alternatively, we might simply set childID to
          // min(childID,numParticles-1) - then in the worst case we'll
          // doubly intersect this one particle, which shouldn't hurt
          // (and is pretty unlikely)
          (nodeID >= numParticles)
          ) {
        break;
      }
#endif
      
      // unmasked {
      //   t_in   = t_nearChild_in;
      //   t_out  = t_nearChild_out;
      // }
      // dim    = nextDim;
      continue;
    }
    // ------------------------------------------------------------------
    // couldn't go down any further; pop a node from stack
    // ------------------------------------------------------------------
    
    // print("Popping...\n");
    while (1) {
      // pop as long as we have to ... or until nothing is left to pop.
      if (stackPtr == stack) {
        // print("DONE\n");
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

      // print("pop - intersect %\n",stackPtr->sphereID);
      // intersect the actual node...
      PartiKDGeometry_intersectPrim(THIS,stackPtr->sphereID,ray);
      if (isShadowRay && ray.primID >= 0) return;

      // do the distance test again, we might just have shortened the ray...
      unmasked { t_out  = min(t_out,ray.t); }
      nodeID = min(stackPtr->farChildID,numParticles-1);
      dim    = stackPtr->dim;
      if (any(t_in < t_out)) {
        break;
      }
    }
  }
}
        

inline void PartiKDGeometry_kd_intersect_packet(uniform PartiKDGeometry *uniform THIS,
                                                varying Ray &ray,
                                                uniform size_t primID)
{
  float t_in = ray.t0, t_out = ray.t;
  boxtest(ray,THIS->sphereBounds,t_in,t_out);
  
  if (t_in >= t_out)
    return;

  const varying float rdir[3] = { rcp(ray.dir.x),rcp(ray.dir.y),rcp(ray.dir.z) };
  const varying float org[3]  = { ray.org.x, ray.org.y, ray.org.z };
// #if 1
  uniform size_t dir_sign[3];
  if (ray.dir.z > 0.f) {
    dir_sign[2] = 0;
    if (ray.dir.y > 0.f) {
      dir_sign[1] = 0;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,false);
      } else {
        dir_sign[0] = 1;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,false);
      }
    } else {
      dir_sign[1] = 1;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,false);
      } else {
        dir_sign[0] = 1;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,false);
      }
    }
  } else {
    dir_sign[2] = 1;
    if (ray.dir.y > 0.f) {
      dir_sign[1] = 0;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,false);
      } else {
        dir_sign[0] = 1;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,false);
      }
    } else {
      dir_sign[1] = 1;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,false);
      } else {
        dir_sign[0] = 1;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,false);
      }
    }
  }
// #else
  // varying int   dir_sign_z = (ray.dir.z < 0.f); 
  // varying int   dir_sign_y = (ray.dir.y < 0.f); 
  // varying int   dir_sign_x = (ray.dir.x < 0.f); 
  // for (uniform int signZ=0;signZ<2;signZ++) {
  //   if (dir_sign_z != signZ) continue;
  //   for (uniform int signY=0;signY<2;signY++) {
  //     if (dir_sign_y != signY) continue;
  //     for (uniform int signX=0;signX<2;signX++) {
  //       if (dir_sign_x != signX) continue;
  //       uniform int dir_sign[3] = { signX,signY,signZ };
  //       PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign);
  //     }
  //   }
  // }
// #endif
}


void PartiKDGeometry_kd_occluded_packet(uniform PartiKDGeometry *uniform THIS,
                                     varying Ray &ray,
                                     uniform size_t primID)
{
  float t_in = ray.t0, t_out = ray.t;
  boxtest(ray,THIS->sphereBounds,t_in,t_out);

  if (t_out < t_in)
    return;
  
  const varying float rdir[3] = { rcp(ray.dir.x),rcp(ray.dir.y),rcp(ray.dir.z) };
  const varying float org[3]  = { ray.org.x, ray.org.y, ray.org.z };

  uniform size_t dir_sign[3];
  if (ray.dir.z > 0.f) {
    dir_sign[2] = 0;
    if (ray.dir.y > 0.f) {
      dir_sign[1] = 0;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,true);
      } else {
        dir_sign[0] = 1;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,true);
      }
    } else {
      dir_sign[1] = 1;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,true);
      } else {
        dir_sign[0] = 1;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,true);
      }
    }
  } else {
    dir_sign[2] = 1;
    if (ray.dir.y > 0.f) {
      dir_sign[1] = 0;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,true);
      } else {
        dir_sign[0] = 1;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,true);
      }
    } else {
      dir_sign[1] = 1;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,true);
      } else {
        dir_sign[0] = 1;
        PartiKDGeometry_packet(THIS,ray,rdir,org,t_in,t_out,dir_sign,true);
      }
    }
  }
}

struct PartiKDStackEntryPacket {
  varying float t_in, t_out;
  uniform size_t nodeID;
  IF_HAS_DEPTH(uniform size_t depth;)
};


void PartiKDGeometry_intersect_packet(uniform PartiKDGeometry *uniform THIS,
                                      varying Ray &ray,
                                      uniform size_t primID)
{
  float t_in = ray.t0, t_out = ray.t;
  boxtest(ray,THIS->sphereBounds,t_in,t_out);
  
  if (t_out < t_in) return;

  PartiKDStackEntryPacket stack[64];
  uniform int32       stackPtr = 0;
  IF_HAS_DEPTH(uniform size_t      depth    = 0);
  float org[3], rdir[3];
  org[0] = ray.org.x;
  org[1] = ray.org.y;
  org[2] = ray.org.z;
  rdir[0] = rcp(ray.dir.x);
  rdir[1] = rcp(ray.dir.y);
  rdir[2] = rcp(ray.dir.z);

  uniform size_t nodeID = 0;
  const uniform uint32 *uniform nodeBinBits = THIS->innerNode_attributeMask; //THIS->innerNode_binBits;
  while (1) {
    while (1) {
      // print("nodeID % depth %\n",nodeID,depth);
      if (t_out < t_in) break;

#if DIM_FROM_DEPTH
      const uniform size_t dim = depth % 3; //THIS->innerNode_dimBits;
#else
      INT3 *uniform intPtr = (INT3 *uniform)THIS->particle;
      const uniform size_t dim = intPtr[nodeID].x & 3;
      // print("dim %\n",dim);
#endif
      const uniform float plane_coord = THIS->particle[nodeID].position[dim];
      
      const float rdir_dim = rdir[dim];
      const float org_dim  = org[dim];

      const float t_plane_0 = (plane_coord - org_dim + THIS->particleRadius) * rdir_dim;
      const float t_plane_1 = (plane_coord - org_dim - THIS->particleRadius) * rdir_dim;
      // const float t_plane_0 = (plane_coord - org[dim] + THIS->particleRadius) * rdir[dim];
      // const float t_plane_1 = (plane_coord - org[dim] - THIS->particleRadius) * rdir[dim];

      if (nodeID >= THIS->numInnerNodes) {
        // this is a leaf ...
        if (!all(min(t_plane_0,t_plane_1) >= t_out)) 
          PartiKDGeometry_intersectPrim(THIS,nodeID,ray);
        break; // pop
      } else {
        if (nodeBinBits != NULL) {
          const uniform uint32 binBits = nodeBinBits[nodeID];
          if ((THIS->transferFunction_activeBinBits & binBits) == 0) break;
        }
        
        if (!all(min(t_plane_0,t_plane_1) >= t_out)) 
          PartiKDGeometry_intersectPrim(THIS,nodeID,ray);
        t_out = min(t_out,ray.t);
        
        float t_in_0 = t_in;
        float t_out_0 = t_out;

        float t_in_1 = t_in;
        float t_out_1 = t_out;
        uniform bool goNode1First = false;
        if (rdir[dim] > 0.f) {
          t_out_0 = min(t_out_0,t_plane_0);
          t_in_1 = max(t_in_1,t_plane_1);
        } else {
          t_out_1 = min(t_out_1,t_plane_1);
          t_in_0 = max(t_in_0,t_plane_0);
          goNode1First = true;
        }
        uniform bool goNode0 = any(t_out_0 > t_in_0);
        uniform bool goNode1 = any(t_out_1 > t_in_1);
        if (goNode0) {
          if (goNode1) {
            if (goNode1First) {
              unmasked { stack[stackPtr].t_in  = 1e20f; }
              unmasked { stack[stackPtr].t_out = -1e20f; }
              stack[stackPtr].t_in  = t_in_0;
              stack[stackPtr].t_out = t_out_0;
              stack[stackPtr].nodeID = 2*nodeID+1;
              IF_HAS_DEPTH(stack[stackPtr].depth = depth+1);
              stackPtr++;
              t_in = t_in_1;
              t_out = t_out_1;
              nodeID = 2*nodeID+2;
              IF_HAS_DEPTH(++depth);
            } else {
              unmasked { stack[stackPtr].t_in  = 1e20f; }
              unmasked { stack[stackPtr].t_out = -1e20f; }
              stack[stackPtr].t_in  = t_in_1;
              stack[stackPtr].t_out = t_out_1;
              stack[stackPtr].nodeID = 2*nodeID+2;
              IF_HAS_DEPTH(stack[stackPtr].depth = depth+1);
              stackPtr++;
              t_in = t_in_0;
              t_out = t_out_0;
              nodeID = 2*nodeID+1;
              IF_HAS_DEPTH(++depth);
            }
            continue;
          } else {
            t_in = t_in_0;
            t_out = t_out_0;
            nodeID = 2*nodeID+1;
            IF_HAS_DEPTH(++depth);
            continue;
          }
        } else {
          if (goNode1) {
            t_in = t_in_1;
            t_out = t_out_1;
            nodeID = 2*nodeID+2;
            IF_HAS_DEPTH(++depth);
            continue;
          } else {
            break; // pop
          }
        }
      }
    }
    // TODO: pop
    while (1) {
      if (stackPtr == 0) return;
      --stackPtr;
      t_in = stack[stackPtr].t_in;
      t_out = min(stack[stackPtr].t_out,ray.t);
      if (all(t_in > t_out)) continue;
      nodeID = stack[stackPtr].nodeID;
      IF_HAS_DEPTH(depth  = stack[stackPtr].depth);
      break;
    };
  }
}


void PartiKDGeometry_occluded_packet(uniform PartiKDGeometry *uniform THIS,
                                     varying Ray &ray,
                                     uniform size_t primID)
{
#if 1
  PartiKDGeometry_intersect_packet(THIS,ray,primID);
#else
  float t_in = ray.t0, t_out = ray.t;
  boxtest(ray,THIS->sphereBounds,t_in,t_out);

  
  if (t_out < t_in) return;

  PartiKDStackEntryPacket stack[64];
  uniform int32 stackPtr = 0;
  uniform int32 depth = 0;

  float org[3], dir[3];
  org[0] = ray.org.x;
  org[1] = ray.org.y;
  org[2] = ray.org.z;
  dir[0] = ray.dir.x;
  dir[1] = ray.dir.y;
  dir[2] = ray.dir.z;

  uniform size_t nodeID = 0;
  const unsigned uint32 *uniform nodeBinBits = THIS->innerNode_attributeMask; //THIS->innerNode_binBits;
  while (1) {
    while (1) {
      if (t_out < t_in) break;
      if (nodeID >= THIS->numInnerNodes) {
        // this is a leaf ...
        if (PartiKDGeometry_intersectPrim(THIS,nodeID,ray))
          return;
        break; // pop
      } else {
        if (nodeBinBits) {
          const uniform uint32 binBits = nodeBinBits[nodeID];
          if ((THIS->transferFunction_activeBinBits & binBits) == 0) break;
        }
        
        if (PartiKDGeometry_intersectPrim(THIS,nodeID,ray))
          return;
        t_out = min(t_out,ray.t);
        
        const uniform size_t dim = depth % 3; //THIS->innerNodeInfo[nodeID] >> 30;
        const uniform float plane_coord = THIS->particle[nodeID].position[dim];

        const float t_plane_0 = (plane_coord + THIS->particleRadius - org[dim]) * rcp(dir[dim]);
        const float t_plane_1 = (plane_coord - THIS->particleRadius - org[dim]) * rcp(dir[dim]);
        
        float t_in_0 = t_in;
        float t_out_0 = t_out;

        float t_in_1 = t_in;
        float t_out_1 = t_out;
        uniform bool goNode1First = false;
        if (dir[dim] > 0.f) {
          t_out_0 = min(t_out_0,t_plane_0);
          t_in_1 = max(t_in_1,t_plane_1);
        } else {
          t_out_1 = min(t_out_1,t_plane_1);
          t_in_0 = max(t_in_0,t_plane_0);
          goNode1First = true;
        }
        uniform bool goNode0 = any(t_out_0 >= t_in_0);
        uniform bool goNode1 = any(t_out_1 >= t_in_1);
        if (goNode0) {
          if (goNode1) {
            if (goNode1First) {
              unmasked { stack[stackPtr].t_in  = 1e20f; }
              unmasked { stack[stackPtr].t_out = -1e20f; }
              stack[stackPtr].t_in  = t_in_0;
              stack[stackPtr].t_out = t_out_0;
              stack[stackPtr].nodeID = 2*nodeID+1;
              stack[stackPtr].depth = depth+1;
              stackPtr++;
              depth = depth + 1;
              t_in = t_in_1;
              t_out = t_out_1;
              nodeID = 2*nodeID+2;
            } else {
              unmasked { stack[stackPtr].t_in  = 1e20f; }
              unmasked { stack[stackPtr].t_out = -1e20f; }
              stack[stackPtr].t_in  = t_in_1;
              stack[stackPtr].t_out = t_out_1;
              stack[stackPtr].nodeID = 2*nodeID+2;
              stack[stackPtr].depth = depth+1;
              stackPtr++;
              depth = depth + 1;
              t_in = t_in_0;
              t_out = t_out_0;
              nodeID = 2*nodeID+1;
            }
            continue;
          } else {
            t_in = t_in_0;
            t_out = t_out_0;
            nodeID = 2*nodeID+1;
            depth = depth + 1;
            continue;
          }
        } else {
          if (goNode1) {
            t_in = t_in_1;
            t_out = t_out_1;
            nodeID = 2*nodeID+2;
            depth = depth + 1;
            continue;
          } else {
            break; // pop
          }
        }
      }
    }
    // TODO: pop
    while (1) {
      if (stackPtr == 0) return;
      --stackPtr;
      t_in = stack[stackPtr].t_in;
      t_out = min(stack[stackPtr].t_out,ray.t);
      if (all(t_in > t_out)) continue;
      nodeID = stack[stackPtr].nodeID;
      depth = stack[stackPtr].depth;
      break;
    };
  }
#endif
}
#endif


export void *uniform PartiKDGeometry_create(void           *uniform cppEquivalent)
{
  uniform PartiKDGeometry *uniform geom = uniform new uniform PartiKDGeometry;
  Geometry_Constructor(&geom->geometry,cppEquivalent,
                       PartiKDGeometry_postIntersect,
                       NULL,0,NULL);
  return geom;
}

export void PartiKDGeometry_updateTransferFunction(void           *uniform _THIS,
                                                   void           *uniform _transferFunction)
{
  PartiKDGeometry *uniform THIS = (PartiKDGeometry *uniform)_THIS;
  LinearTransferFunction *uniform transferFunction = (LinearTransferFunction *uniform)_transferFunction;
  THIS->transferFunction_activeBinBits = 0;
  for (uniform int i=0;i<32;i++) {
    uniform float a0 = i/32.f;
    uniform float a1 = (i+1)/32.f - 1e-5f;
    uniform float alphaRange = getMaxOpacityForRange(transferFunction,a0,a1);
    if (alphaRange >= .5f)
      THIS->transferFunction_activeBinBits |= (1UL << i);
  }
  // print("#osp:pkd: new range bits: %\n",(void*uniform )THIS->transferFunction_activeBinBits);
}

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

  uniform uint32 geomID = rtcNewUserGeometry(model->embreeSceneHandle,1); //numSpheres);
  
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
  // geom->activeBinBits = 0xffffffff;

  geom->transferFunction  = (TransferFunction *uniform)transferFunction;

  rtcSetUserData(model->embreeSceneHandle,geomID,geom);
  rtcSetBoundsFunction(model->embreeSceneHandle,geomID,
                       (uniform RTCBoundsFunc)&PartiKDGeometry_bounds);
#if USE_NAIVE_SPMD_TRAVERSAL
  rtcSetIntersectFunction(model->embreeSceneHandle,geomID,
                          (uniform RTCIntersectFuncVarying)&PartiKDGeometry_intersect_naive);
  rtcSetOccludedFunction(model->embreeSceneHandle,geomID,
                         (uniform RTCOccludedFuncVarying)&PartiKDGeometry_occluded_naive);
#else
# if 1
  rtcSetIntersectFunction(model->embreeSceneHandle,geomID,
                          (uniform RTCIntersectFuncVarying)&PartiKDGeometry_kd_intersect_packet);
  rtcSetOccludedFunction(model->embreeSceneHandle,geomID,
                         (uniform RTCOccludedFuncVarying)&PartiKDGeometry_kd_occluded_packet);
# else
  rtcSetIntersectFunction(model->embreeSceneHandle,geomID,
                          (uniform RTCIntersectFuncVarying)&PartiKDGeometry_intersect_packet);

  rtcSetOccludedFunction(model->embreeSceneHandle,geomID,
                         (uniform RTCOccludedFuncVarying)&PartiKDGeometry_occluded_packet);
# endif
#endif

  if (transferFunction) PartiKDGeometry_updateTransferFunction(geom,transferFunction);
}
