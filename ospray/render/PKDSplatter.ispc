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
#include "ospray/common/Box.ih"
// this module
#include "../PKDGeometry.ih"

struct PKDSplatter
{
  Renderer inherited;
  float radius;
  float weight;
  uniform box3f sphereBounds;
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

inline void splatParticle(const uniform vec3f &pos,
                          const uniform float radius,
                          const uniform float splatWeight,
                          const varying vec3f &nDir,
                          varying Ray &ray,
                          float &sample)
{
  vec3f v = pos - ray.org;
  vec3f pos_proj_on_dir = (dot(nDir,v)/(dot(v,v)+1e20f)) * nDir;
  vec3f shortest_vec_to_pos = pos - pos_proj_on_dir;
  float dist2 = dot(shortest_vec_to_pos,shortest_vec_to_pos);
  if (dist2 > radius*radius) 
    return;
  float dist = sqrtf(dist2);
  float splatValue = splatWeight * (1.f-dist/radius);
  sample += splatValue;
  float z = sqrtf(dot(pos_proj_on_dir,pos_proj_on_dir));
  ray.t = min(ray.t,z);
  ray.primID = 0;
}

struct SplatStackEntry {
  varying float t0, t1;
  uniform uint64 nodeID;
};

void PKD_splatParticles(uniform PKDSplatter *uniform self,
                        varying Ray &ray,
                        varying float &sample)
{
  float t0 = ray.t0, t1 = ray.t;
  boxtest(ray,self->sphereBounds,t0,t1);
  
  if (t1 < t0)
    return;
  uniform uint64 nodeID = 0;
  uniform SplatStackEntry stackBase[64];
  uniform SplatStackEntry *uniform stackPtr = stackBase;
  uniform PartiKDGeometry *uniform pkd = self->pkd;
  const uniform float radius = self->radius;
  const uniform float weight = self->weight;
  const uniform uint64 numInnerNodes = pkd->numInnerNodes;
  const uniform uint64 numParticles  = pkd->numParticles;
  // const uniform uint64 *uniform const particle = self->particle;

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
  const vec3f nDir = normalize(ray.dir);

  while (1)  {
    while (1) {
      // uniform uint32 pBits = particle[nodeID];
      uniform Particle p;
      getParticle(self->pkd,p,nodeID);
      splatParticle(make_vec3f(p.pos[0],p.pos[1],p.pos[2]),
                    radius,weight,nDir,ray,sample);
      if (sample >= 1.f) return;
      
      if (nodeID >= numInnerNodes) 
        break;

      const float org_to_node_dim = p.pos[p.dim] - org[p.dim];
      const float t_plane_0  = (org_to_node_dim - radius) * rdir[p.dim];
      const float t_plane_1  = (org_to_node_dim + radius) * rdir[p.dim];
      
      float t0l = t0;
      float t1l = t1;
      float t0r = t0;
      float t1r = t1;
      if (rdir[p.dim] < 0.f) {
        t0l = max(t0l,t_plane_1);
        t1r = min(t1r,t_plane_0);
      } else {
        t0r = max(t0r,t_plane_0);
        t1l = min(t1l,t_plane_1);
      }
      uniform bool any_l = any(t0l < t1l);
      uniform bool any_r = any(t0r < t1r);
      uniform uint64 node_l = 2*nodeID+1;
      uniform uint64 node_r = node_l+1;

      if (any_l) {
        if (any_r) {
          if (any((t0l < t1l) & (t0r < t1r) & (rdir[p.dim] < 0))) {
            unmasked {
              stackPtr->t0 = t0l;
              stackPtr->t1 = t1l;
            }
            stackPtr->nodeID = node_l;
            ++stackPtr;
            nodeID = node_r;
            t0 = t0r;
            t1 = t1r;
          } else {
            unmasked {
              stackPtr->t0 = t0r;
              stackPtr->t1 = t1r;
            }
            stackPtr->nodeID = node_r;
            ++stackPtr;
            nodeID = node_l;
            t0 = t0l;
            t1 = t1l;
          }
        } else {
          nodeID = node_l;
          t0 = t0l;
          t1 = t1l;
        }
      } else {
        if (any_r) {
          nodeID = node_r;
          t0 = t0r;
          t1 = t1r;
        } else {
          break;
        }
      }
    }
    // pop
    while (1) {
      --stackPtr;
      if (stackPtr <= stackBase) return;
      unmasked {
        t0 = stackPtr->t0;
        t1 = stackPtr->t1;
      }
      nodeID = stackPtr->nodeID;
      if (none(t0 < t1)) continue;
      break;
    }
  }
}

void PKDSplatter_renderSample(uniform Renderer *uniform _renderer,
                                   varying ScreenSample &sample)
{
  uniform PKDSplatter *uniform self = (uniform PKDSplatter *uniform)_renderer;
  
  sample.alpha = 1.f;
  float splatVal = 0.f;
  PKD_splatParticles(self,sample.ray,splatVal);
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
  self->pkd = pkd;
  self->radius = radius;
  self->weight = weight;
}                                                                     

export void *uniform PKDSplatter_create(void *uniform cppE)                     
{                                                                     
  uniform PKDSplatter *uniform self                           
    = uniform new uniform PKDSplatter;                            
  Renderer_constructor(&self->inherited,cppE,NULL,NULL);          
  self->inherited.renderSample = PKDSplatter_renderSample;                            
  return self;                                                  
}                                                                     

export void PKDSplatter_destroy(void *uniform _renderer)
{
  uniform PKDSplatter *uniform renderer = (uniform PKDSplatter *uniform)_renderer;
  delete renderer;
}
