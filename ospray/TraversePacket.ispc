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


// uniform int rayID = 0;

#define LOD 0

inline varying bool PartiKDGeometry_intersectPrim(PartiKDGeometry *uniform self,
                                                  uniform Particle &p,
                                                  uniform primID_t primID,
                                                  varying Ray &ray)
{
  // uniform bool dbg = false; //(rayID == 338);
  
  // if (dbg) print("ENTER ISEC primID %\n",primID);
  // typecast "implicit self" pointer to the proper geometry type

  // PartiKDGeometry *uniform self = (PartiKDGeometry *uniform)geomPtr;
// #if PRIMID64
//   const uniform int64 offset = 3*primID;
//   const uniform float *uniform pos = &self->particle[0].position[0];
//   pos += offset;
// #else
//   const uniform float *uniform pos = &self->particle[primID].position[0];
// #endif
//   if (dbg) print("ISEC pos0 %\n",self->particle);
//   if (dbg) print("ISEC pos ptr %\n",pos);
  // read sphere members required for intersection test
  // if (dbg) print("got radius %\n",radius);
  // // uniform vec3f center = (uniform vec3f &)self->particle[primID].position;
  // if (dbg) print("getting pos...\n");
  // const vec3f center = make_vec3f((varying float)pos[0],pos[1],pos[2]);
  // if (dbg) print("GOT pos...\n");

  // uniform vec3f center;
  // uniform uint32 dim;
  // getParticle(self,primID,center,dim);
  
  // perform first half of intersection test ....
  const vec3f A = make_vec3f(p.pos[0],p.pos[1],p.pos[2]) - ray.org;

  const float a = dot(ray.dir,ray.dir);
  const float b = -2.f*dot(ray.dir,A);
	const float AA = dot(A,A);
#if LOD
  const float radius = self->particleRadius * max(1.f, (1.f/16.f) * sqrt(sqrt(AA)));
#else	
  const float radius = self->particleRadius;
#endif	
	const float c = AA-radius*radius;
  
  const float radical = b*b-4.f*a*c;
  if (radical < 0.f) return false;

  // if (dbg) print("ISEC1\n");
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

  // if (dbg) print("ISEC2\n");
  // do attribute alpha test, if both attribute and transfer fct are set
  if ((self->attribute!=NULL) & (self->transferFunction!=NULL)) {
    // -------------------------------------------------------
    // do attribute test
    uniform float attrib = self->attribute[primID];

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

  // if (dbg) print("ISEC3\n");
  // found a hit - store it
#if PRIMID64
  ray.primID = primID;
  ray.primID_hi64 = primID >> 32;
#else
  ray.primID = primID;
#endif
  ray.geomID = self->geometry.geomID;
  ray.t = t_in;
  ray.Ng = ray.t*ray.dir - A;
  // if (dbg) print("LEAVE ISEC HIT\n");
  return true;
}

struct ThreePhaseStackEntry {
  varying float t_in, t_out, t_sphere_out;
  uniform primID_t sphereID;
  uniform primID_t farChildID;
// #if DIM_FROM_DEPTH
//   uniform int32  dim;
// #endif
};

inline void pkd_traverse_packet(uniform PartiKDGeometry *uniform self,
                                varying Ray &ray,
                                const varying float rdir[3], 
                                const varying float org[3],
                                const varying float t_in_0, 
                                const varying float t_out_0,
                                const uniform size_t dir_sign[3],
                                const uniform bool isShadowRay
                                )
{
  // ++rayID;
  // print("Ray %\n",rayID);
  // uniform bool dbg = false; //(rayID == 338);

  varying ThreePhaseStackEntry stack[64];
  varying ThreePhaseStackEntry *uniform stackPtr = stack;
  
  uniform primID_t nodeID = 0;
  uniform size_t dim    = 0;
  
  float t_in = t_in_0;
  float t_out = t_out_0;
  const float radius = self->particleRadius;
  const uniform primID_t numInnerNodes = self->numInnerNodes;
  const uniform primID_t numParticles  = self->numParticles;
  const uniform PKDParticle *uniform const particle = self->particle;
  // if (dbg) print("ENTER TRAVERSAL\n");
  uniform Particle p;
  while (1) {
    // ------------------------------------------------------------------
    // do traversal step(s) as long as possible
    // ------------------------------------------------------------------
    // if (dbg) print("TRAV %\n",nodeID);
    while (1) {    
      // if (dbg) print("DOWN %\n",nodeID);

      if (t_in > t_out) break;

      getParticle(self,p,nodeID);

#if LOD			
			const float dist_lod = max(1.f, (1.f/16.f) * sqrt(t_in));
			if (nodeID > 4096)
			{
				uniform Particle parent;
				getParticle(self,parent,(nodeID-1)>>1);
				uniform vec3f d = make_vec3f(p.pos[0] - parent.pos[0], p.pos[1] - parent.pos[1], p.pos[2] - parent.pos[2]);
				//if distance is < 1 pixel, previous point was "good enough"; break;
				if (dot(d,d) < dist_lod)
						break;
			}
#endif
			
      if (nodeID >= numInnerNodes) {
        // this is a leaf node - can't to to a leaf, anyway. Intersect
        // the prim, and be done with it.
        // if (dbg) print("LEAFISEC0\n");
        PartiKDGeometry_intersectPrim(self,p,nodeID,ray);
        // if (dbg) print("LEAFISEC1\n");
        if (isShadowRay && ray.primID >= 0) return;
        break;
      } 

      if (self->innerNode_attributeMask) {
        const uniform uint32 nodeAttrBits = self->innerNode_attributeMask[nodeID];
        if ((nodeAttrBits & self->transferFunction_activeBinBits) == 0)
          break;
      }

// #if !DIM_FROM_DEPTH
      // INT3 *uniform intPtr = (INT3 *uniform)self->particle;
      // dim = intPtr[nodeID].x & 3;
      dim = p.dim;
// #endif

      const uniform size_t sign = dir_sign[dim];
			
      // ------------------------------------------------------------------
      // traversal step: compute distance, then compute intervals for front and back side
      // ------------------------------------------------------------------
      const float org_to_node_dim = p.pos[dim] - org[dim];
#if LOD
      const float t_plane_0  = (org_to_node_dim - radius * dist_lod) * rdir[dim];
      const float t_plane_1  = (org_to_node_dim + radius * dist_lod) * rdir[dim];
#else
      const float t_plane_0  = (org_to_node_dim - radius) * rdir[dim];
      const float t_plane_1  = (org_to_node_dim + radius) * rdir[dim];			
#endif
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

      // if (dbg) print("nodeID a0 %\n",nodeID);
      nodeID = min(2*nodeID+1+sign,numParticles-1);
      // if (dbg) print("nodeID a1 %\n",nodeID);
      continue;
    }
    // ------------------------------------------------------------------
    // couldn't go down any further; pop a node from stack
    // ------------------------------------------------------------------
    while (1) {
      // if (dbg) print("POP\n");
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
        uniform Particle p;
        getParticle(self,p,stackPtr->sphereID);
        PartiKDGeometry_intersectPrim(self,p,stackPtr->sphereID,ray);
        if (isShadowRay && ray.primID >= 0) return;
      } 
      
      // do the distance test again, we might just have shortened the ray...
      unmasked { t_out  = min(t_out,ray.t); }
      // if (dbg) print("nodeID b0 %\n",nodeID);
      nodeID = min(stackPtr->farChildID,numParticles-1);
      // if (dbg) print("nodeID b1 %\n",nodeID);
#if DIM_FROM_DEPTH
      dim    = stackPtr->dim;
#endif
      break;
    }
  }
  // if (dbg) print("DONE TRAVERSAL\n");
}

/*! generic traverse/occluded function that splits the packet into
  subpackets of equal sige, and then calls the appropiate
  constant-sign traverse function. this method works for both shadow
  and primary rays, as indicated by the 'isShadowRay' flag */
inline void pkd_traverse_packet(uniform PartiKDGeometry *uniform self,
                                varying Ray &ray,
                                uniform size_t primID,
                                uniform bool isShadowRay)
{
  float t_in = ray.t0, t_out = ray.t;
  boxtest(ray,self->sphereBounds,t_in,t_out);

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
        pkd_traverse_packet(self,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(self,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      }
    } else {
      dir_sign[1] = 1;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_traverse_packet(self,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(self,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      }
    }
  } else {
    dir_sign[2] = 1;
    if (ray.dir.y > 0.f) {
      dir_sign[1] = 0;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_traverse_packet(self,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(self,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      }
    } else {
      dir_sign[1] = 1;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_traverse_packet(self,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(self,ray,rdir,org,t_in,t_out,dir_sign,isShadowRay);
      }
    }
  }
}

/*! the 'virtual' traverse function for a pkd geometry */
void PartiKDGeometry_intersect_packet(uniform PartiKDGeometry *uniform self,
                                      varying Ray &ray,
                                      uniform size_t primID)
{ pkd_traverse_packet(self,ray,primID,false); }

/*! the 'virtual' occluded function for a pkd geometry */
void PartiKDGeometry_occluded_packet(uniform PartiKDGeometry *uniform self,
                                     varying Ray &ray,
                                     uniform size_t primID)
{ pkd_traverse_packet(self,ray,primID,true); }


