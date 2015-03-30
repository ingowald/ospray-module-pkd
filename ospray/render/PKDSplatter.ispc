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
#include "ospray/fb/FrameBuffer.ih"
#include "ospray/camera/PerspectiveCamera.ih"
#include "ospray/common/Model.ih"
#include "ospray/render/util.ih"
#include "ospray/render/Renderer.ih"
#include "ospray/math/bbox.ih"
#include "ospray/common/Box.ih"
// this module
#include "../PKDGeometry.ih"

// if enabled, we're doing some LOD on the tree traversal ... doesn't properly work, yet!
//#define LOD 1


struct PKDSplatter
{
  Renderer inherited;
  float radius;
  float weight;
  PartiKDGeometry *pkd;
};

// inline void decode(uniform uint64 bits, uniform vec3f &pos, uniform int32 dim)
// {
//   uniform int mask = (1<<20)-1;
//   uniform int ix = (bits >> 2) & mask;
//   uniform int iy = (bits >> 22) & mask;
//   uniform int iz = (bits >> 42) & mask;
//   pos.x = ix;
//   pos.y = iy;
//   pos.z = iz;
//   dim = bits & 3;
// }

inline void splatParticle(PKDSplatter *uniform self,
                          const uniform Particle p,
                          varying Ray &ray,
                          const vec3f &nDir,
                          float &sample)
{
  const vec3f pos = make_vec3f(p.pos[0],p.pos[1],p.pos[2]);
  vec3f v = pos - ray.org;
  float l = sqrtf(dot(v,v));
  v = normalize(v);
  float cosAngle = dot(v,nDir);

  vec3f pos_proj_on_dir = ray.org + (l*cosAngle) * nDir;
  vec3f shortest_vec_to_pos = pos - pos_proj_on_dir;
  float dist2 = dot(shortest_vec_to_pos,shortest_vec_to_pos);
  float radius = self->radius;
  float weight = self->weight;
  if (dist2 > radius*radius) 
    return;
  float dist = sqrtf(dist2);
  float splatValue = weight * (1.f-dist/radius);
  sample += splatValue;
  float z = sqrtf(dot(pos_proj_on_dir,pos_proj_on_dir));
  ray.primID = 0;
}

struct SplatStackEntry {
  varying float t0, t1;
  uniform uint64 nodeID;
};



struct ThreePhaseStackEntry {
  varying float t_in, t_out, t_sphere_out;
  uniform primID_t sphereID;
  uniform primID_t farChildID;
// #if DIM_FROM_DEPTH
//   uniform int32  dim;
// #endif
};

inline void pkd_splat_packet(uniform PKDSplatter *uniform self,
                             varying Ray &ray,
                             const varying float rdir[3], 
                             const varying float org[3],
                             const varying float t_in_0, 
                             const varying float t_out_0,
                             const uniform size_t dir_sign[3],
                             varying float &splatValue
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
  const float radius = self->radius;
  const uniform primID_t numInnerNodes = self->pkd->numInnerNodes;
  const uniform primID_t numParticles  = self->pkd->numParticles;
  //  const uniform PKDParticle *uniform const particle = self->pkd->particle;
  // if (dbg) print("ENTER TRAVERSAL\n");
  uniform Particle p;
  const varying vec3f nDir = normalize(ray.dir); //&nDir,
  while (1) {
    // ------------------------------------------------------------------
    // do traversal step(s) as long as possible
    // ------------------------------------------------------------------
    // if (dbg) print("TRAV %\n",nodeID);
    while (1) {    
      // if (dbg) print("DOWN %\n",nodeID);

      if (t_in > t_out) break;

      getParticle(self->pkd,p,nodeID);
      if (nodeID >= numInnerNodes) {
        // this is a leaf node - can't to to a leaf, anyway. Intersect
        // the prim, and be done with it.
        // if (dbg) print("LEAFISEC0\n");
        splatParticle(self,p,ray,nDir,splatValue);
        if (splatValue >= 1.f) return;
        // PartiKDGeometry_intersectPrim(self,p,nodeID,ray);
        // if (dbg) print("LEAFISEC1\n");
        // if (isShadowRay && ray.primID >= 0) return;
        break;
      } 

      // if (self->innerNode_attributeMask) {
      //   const uniform uint32 nodeAttrBits = self->innerNode_attributeMask[nodeID];
      //   if ((nodeAttrBits & self->transferFunction_activeBinBits) == 0)
      //     break;
      // }

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
        getParticle(self->pkd,p,stackPtr->sphereID);
        splatParticle(self,p,ray,nDir,splatValue);
        if (splatValue >= 1.f) return;
        // PartiKDGeometry_intersectPrim(self,p,stackPtr->sphereID,ray);
        // if (isShadowRay && ray.primID >= 0) return;
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

struct BOX3f {
  float lower[3], upper[3];
};

struct LODStackEntry {
  uniform BOX3f    bounds;
  uniform primID_t nodeID;
};




inline vec3f make_vec3f(float f[3]) { return make_vec3f(f[0],f[1],f[2]); }
inline uniform vec3f make_vec3f(uniform float f[3]) { return make_vec3f(f[0],f[1],f[2]); }

inline bool boxtest(uniform BOX3f &box,
                    Ray& ray,
                    float& t0,
                    float& t1,
                    float radius)
{
  vec3f mins = mul(sub(make_vec3f(box.lower)-make_vec3f(radius), ray.org), rcp(ray.dir));
  vec3f maxs = mul(sub(make_vec3f(box.upper)+make_vec3f(radius), ray.org), rcp(ray.dir));
  
  t0 = max(max(ray.t0, 
               min(mins.x,maxs.x)),
           max(min(mins.y,maxs.y),
               min(mins.z,maxs.z)));
  
  t1 = min(min(ray.t, 
               max(mins.x,maxs.x)),
           min(max(mins.y,maxs.y),
               max(mins.z,maxs.z)));

  return t0 <= t1;
}

/*! LOD-based traversal technique ... */
inline void pkd_splat_LOD(uniform PKDSplatter *uniform self,
                          varying Ray &ray,
                          varying float &sample)
{
  // factor for chosing whether to replace a subtree with a LOD-representation
  const float lodFactor = .000001f;
  sample = 0.f;

  const float radius = self->radius;
  const uniform primID_t numInnerNodes = self->pkd->numInnerNodes;
  const uniform primID_t numParticles  = self->pkd->numParticles;

  uniform BOX3f bounds;
  bounds.lower[0] = self->pkd->sphereBounds.lower.x;
  bounds.lower[1] = self->pkd->sphereBounds.lower.y;
  bounds.lower[2] = self->pkd->sphereBounds.lower.z;
  bounds.upper[0] = self->pkd->sphereBounds.upper.x;
  bounds.upper[1] = self->pkd->sphereBounds.upper.y;
  bounds.upper[2] = self->pkd->sphereBounds.upper.z;

  uniform LODStackEntry nodeStack[128];
  nodeStack[0].bounds = bounds;
  nodeStack[0].nodeID = 0;
  uniform int stackPtr = 1;
  

  uniform primID_t nodeID = 0;

  const varying vec3f nDir = normalize(ray.dir); //&nDir,
  const float ray_dir[3] = { ray.dir.x,ray.dir.y,ray.dir.z };
  uniform Particle p;

  while (stackPtr > 0) {
    --stackPtr;
    nodeID = nodeStack[stackPtr].nodeID;
    if (nodeID >= numParticles)
      continue;

    bounds = nodeStack[stackPtr].bounds;
    float t0, t1;
    if (!boxtest(bounds,ray,t0,t1,radius)) 
      continue;

    const uniform float width = min(min(bounds.upper[0]-bounds.lower[0],
                                        bounds.upper[1]-bounds.lower[1]),
                                    bounds.upper[2]-bounds.lower[2]);
    // const uniform float vec3f center = make_vec3f(0.5f*(bounds.lower[0]+bounds.upper[0]),
    //                                               0.5f*(bounds.lower[1]+bounds.upper[1]),
    //                                               0.5f*(bounds.lower[2]+bounds.upper[2]));
    // const uniform float dist = max(1e-6f,length(center - ray.org));
    const float dist = t0;

    if (width < lodFactor * dist) {
      // replace by LOD effect ...
      uniform int depth = stackPtr;
      float numParticlesInSubtree = ((float)(numParticles)) / ((float)(1<<depth));
      // assume we're 50% of power on average
      sample += numParticlesInSubtree * .5f * self->weight;
      //      TODO;
    } else {
      getParticle(self->pkd,p,nodeID);
      splatParticle(self,p,ray,nDir,sample);
      
      uniform BOX3f lBounds = bounds;
      uniform BOX3f rBounds = bounds;
      lBounds.upper[p.dim] = rBounds.lower[p.dim] = p.pos[p.dim];
      if (any(ray_dir[p.dim] > 0.f)) {
        nodeStack[stackPtr+0].bounds = lBounds;
        nodeStack[stackPtr+0].nodeID = 2*nodeID+1;
        nodeStack[stackPtr+1].bounds = rBounds;
        nodeStack[stackPtr+1].nodeID = 2*nodeID+2;
      } else {
        nodeStack[stackPtr+1].bounds = lBounds;
        nodeStack[stackPtr+1].nodeID = 2*nodeID+1;
        nodeStack[stackPtr+0].bounds = rBounds;
        nodeStack[stackPtr+0].nodeID = 2*nodeID+2;
      }
      stackPtr += 2;
    }

  }

}

/*! generic traverse/occluded function that splits the packet into
  subpackets of equal sige, and then calls the appropiate
  constant-sign traverse function. this method works for both shadow
  and primary rays, as indicated by the 'isShadowRay' flag */
inline void pkd_splat_packet(uniform PKDSplatter *uniform self,
                             varying Ray &ray,
                             varying float &sample)
{
  float t_in = ray.t0, t_out = ray.t;
  boxtest(ray,self->pkd->sphereBounds,t_in,t_out);

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
        pkd_splat_packet(self,ray,rdir,org,t_in,t_out,dir_sign,sample);
      } else {
        dir_sign[0] = 1;
        pkd_splat_packet(self,ray,rdir,org,t_in,t_out,dir_sign,sample);
      }
    } else {
      dir_sign[1] = 1;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_splat_packet(self,ray,rdir,org,t_in,t_out,dir_sign,sample);
      } else {
        dir_sign[0] = 1;
        pkd_splat_packet(self,ray,rdir,org,t_in,t_out,dir_sign,sample);
      }
    }
  } else {
    dir_sign[2] = 1;
    if (ray.dir.y > 0.f) {
      dir_sign[1] = 0;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_splat_packet(self,ray,rdir,org,t_in,t_out,dir_sign,sample);
      } else {
        dir_sign[0] = 1;
        pkd_splat_packet(self,ray,rdir,org,t_in,t_out,dir_sign,sample);
      }
    } else {
      dir_sign[1] = 1;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_splat_packet(self,ray,rdir,org,t_in,t_out,dir_sign,sample);
      } else {
        dir_sign[0] = 1;
        pkd_splat_packet(self,ray,rdir,org,t_in,t_out,dir_sign,sample);
      }
    }
  }
}




void PKD_splatParticles(uniform PKDSplatter *uniform self,
                        varying Ray &ray,
                        varying float &sample)
{
#if LOD
  pkd_splat_LOD(self,ray,sample);
#else
  pkd_splat_packet(self,ray,sample);
#endif

// //   float t0 = ray.t0, t1 = ray.t;
// //   uniform PartiKDGeometry *uniform pkd = self->pkd;
// //   boxtest(ray,pkd->sphereBounds,t0,t1);

// //   if (t1 < t0)
// //     return;

// //   uniform uint64 nodeID = 0;
// //   uniform SplatStackEntry stackBase[64];
// //   uniform SplatStackEntry *uniform stackPtr = stackBase;
// //   const uniform float radius = self->radius;
// //   const uniform float weight = self->weight;
// //   const uniform uint64 numInnerNodes = pkd->numInnerNodes;
// //   const uniform uint64 numParticles  = pkd->numParticles;
// //   // const uniform uint64 *uniform const particle = self->particle;

// //   const varying float rdir[3] = { 
// //     safe_rcp(ray.dir.x),
// //     safe_rcp(ray.dir.y),
// //     safe_rcp(ray.dir.z) 
// //   };
// //   const varying float org[3]  = { 
// //     ray.org.x, 
// //     ray.org.y, 
// //     ray.org.z 
// //   };
// //   const vec3f nDir = normalize(ray.dir);

// //   while (1)  {
// //     while (1) {
// //       // uniform uint32 pBits = particle[nodeID];
// //       uniform Particle p;
// //       getParticle(self->pkd,p,nodeID);
// //       splatParticle(make_vec3f(p.pos[0],p.pos[1],p.pos[2]),
// //                     radius,weight,nDir,ray,sample);
// //       if (sample >= 1.f) return;
      
// //       if (nodeID >= numInnerNodes) 
// //         break;

// //       const float org_to_node_dim = p.pos[p.dim] - org[p.dim];
// //       const float t_plane_0  = (org_to_node_dim - radius) * rdir[p.dim];
// //       const float t_plane_1  = (org_to_node_dim + radius) * rdir[p.dim];
      
// //       float t0l = t0;
// //       float t1l = t1;
// //       float t0r = t0;
// //       float t1r = t1;
// //       if (rdir[p.dim] < 0.f) {
// //         t0l = max(t0l,t_plane_1);
// //         t1r = min(t1r,t_plane_0);
// //       } else {
// //         t0r = max(t0r,t_plane_0);
// //         t1l = min(t1l,t_plane_1);
// //       }
// //       uniform bool any_l = any(t0l < t1l);
// //       uniform bool any_r = any(t0r < t1r);
// //       uniform uint64 node_l = 2*nodeID+1;
// //       uniform uint64 node_r = node_l+1;

// //       if (any_l) {
// //         if (any_r) {
// //           if (any((t0l < t1l) & (t0r < t1r) & (rdir[p.dim] < 0))) {
// //             unmasked {
// //               stackPtr->t0 = t0l;
// //               stackPtr->t1 = t1l;
// //             }
// //             stackPtr->nodeID = node_l;
// //             ++stackPtr;
// //             nodeID = node_r;
// //             t0 = t0r;
// //             t1 = t1r;
// //           } else {
// //             unmasked {
// //               stackPtr->t0 = t0r;
// //               stackPtr->t1 = t1r;
// //             }
// //             stackPtr->nodeID = node_r;
// //             ++stackPtr;
// //             nodeID = node_l;
// //             t0 = t0l;
// //             t1 = t1l;
// //           }
// //         } else {
// //           nodeID = node_l;
// //           t0 = t0l;
// //           t1 = t1l;
// //         }
// //       } else {
// //         if (any_r) {
// //           nodeID = node_r;
// //           t0 = t0r;
// //           t1 = t1r;
// //         } else {
// //           break;
// //         }
// //       }
// //     }
// //     // pop
// //     while (1) {
// //       --stackPtr;
// //       if (stackPtr <= stackBase) return;
// //       unmasked {
// //         t0 = stackPtr->t0;
// //         t1 = stackPtr->t1;
// //       }
// //       nodeID = stackPtr->nodeID;
// //       if (none(t0 < t1)) continue;
// //       break;
// //     }
// //   }
// #endif
}

void PKDSplatter_renderSample(uniform Renderer *uniform _renderer,
                                   varying ScreenSample &sample)
{
  uniform PKDSplatter *uniform self = (uniform PKDSplatter *uniform)_renderer;
  
  sample.alpha = 1.f;
  float splatVal = 0.f;
  PKD_splatParticles(self,sample.ray,splatVal);
  splatVal = min(splatVal,1.f);
  sample.z = sample.ray.t;
  
  sample.rgb = make_vec3f(splatVal);
}

export void PKDSplatter_set(void *uniform _self,
                            void *uniform _model,
                            void *uniform _pkd,
                            void *uniform _camera,
                            uniform float radius,
                            uniform float weight)                     
{                                                                     
  PKDSplatter     *uniform self   = (PKDSplatter *uniform)_self;
  PartiKDGeometry *uniform pkd    = (PartiKDGeometry *uniform)_pkd;
  Model           *uniform model  = (Model *uniform)_model;
  Camera          *uniform camera = (uniform Camera *uniform)_camera;

  self->inherited.model = model;
  self->inherited.camera = camera;
  self->pkd = pkd;
  self->radius = radius;
  self->weight = weight;
}                                                                     

export void *uniform PKDSplatter_create(void *uniform cppE)                     
{                                                                     
  uniform PKDSplatter *uniform self                           
    = uniform new uniform PKDSplatter;                            
  Renderer_Constructor(&self->inherited,cppE);          
  self->inherited.renderSample = PKDSplatter_renderSample;                            
  return self;                                                  
}                                                                     
