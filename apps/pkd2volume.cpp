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

#include "../sg/PKD.h"
#include "sg/SceneGraph.h"
// c++11
#include <mutex>

namespace ospray {
  using std::endl;
  using std::cout;
  
  enum { BLOCK_SIZE = 32 };
  
  struct Block {
    Block(const vec3i &blockID, const vec3i &volumeDims)
      : blockID(blockID), 
        lower(blockID*vec3i(BLOCK_SIZE)), 
        upper(min(lower+vec3i(BLOCK_SIZE),volumeDims)), 
        dims(upper-lower)
    {}

    // 3D block ID; in increments of 1
    vec3i blockID;
    // lower an dupper bounds, in final voxel coordinates
    vec3i lower, upper;
    vec3i dims; // dimensions of (ie, num voxels in) block - pretty much 'upper'-'lower
    float voxel[BLOCK_SIZE][BLOCK_SIZE][BLOCK_SIZE];
  };
  
  struct MappedVolume3f {
    MappedVolume3f(const vec3i dims, const std::string &rawFileName) 
      : dims(dims)
    {
      file = fopen(rawFileName.c_str(),"wb");
      if (!file) 
        throw std::runtime_error("could not open file '"+rawFileName+"' for writing");
    }
    
    ~MappedVolume3f() 
    { fclose(file); }
    
    void writeBlock(Block &block)
    {
      std::lock_guard<std::mutex> lock(mutex);
      for (size_t z=0;z<block.dims.z;z++)
        for (size_t y=0;y<block.dims.y;y++) {
          size_t dx = block.dims.x;
          size_t g_x = block.lower.x;
          size_t g_y = block.lower.y+y;
          size_t g_z = block.lower.z+z;

          size_t fileOfs = (g_x + dims.x * (g_y + dims.y * g_z))*sizeof(float);
          fseek(file,fileOfs,SEEK_SET);
          int written = fwrite(&block.voxel[z][y][0],sizeof(float),dx,file);
          if (written != dx) 
            throw std::runtime_error("error writing block data");
        }
    }

    std::mutex mutex;
    FILE *file;
    vec3i dims;
  };

  void usage(const std::string &err = "")
  {
    if (err != "")
      cout << "Error: " << err << endl << endl;
    cout << "Usage:" << endl;
    cout <<"  ./ospPKD2RAW inFileName.pkd -o outFileName -dims x y z --radius r --border b" << endl;
    cout << endl;
    cout << "exiting." << endl << endl;
    exit(0);
  }

  struct Splatter
  {
    vec3f *particle;
    size_t numParticles;
    // splat radius
    float radius;

    box3f bounds;
    float border;

    vec3f getWorldPos(const vec3i &cellID, const vec3i &volumeDims)
    {
      const vec3f unitCellCenter = (vec3f(cellID)+vec3f(.5f)) / vec3f(volumeDims);
      return
        (bounds.lower - vec3f(border))
        + 
        unitCellCenter*(bounds.upper-bounds.lower + 2.f*vec3f(border));
    }

    inline float splatValue(float dist)
    { 
      if (dist >= radius) return 0.f;
      return 1.f - dist / radius;
    }

    float computeSampleRec(const vec3f &samplePos, size_t particleID)
    {
      if (particleID >= numParticles)
        return 0;

      const vec3f &particle = this->particle[particleID];
      int dim = ((int &)particle.x) & 3;
      float plane = (&particle.x)[dim];
      float coord = (&samplePos.x)[dim];

      float value = splatValue(distance(samplePos,particle));

      size_t lChild = 2*particleID+1;
      size_t rChild = 2*particleID+2;
      if (coord >= plane) {
        value += computeSampleRec(samplePos,rChild);
        if (fabsf(coord-plane) < radius)
          value += computeSampleRec(samplePos,lChild);
      } else {
        value += computeSampleRec(samplePos,lChild);
        if (fabsf(coord-plane) < radius)
          value += computeSampleRec(samplePos,rChild);
      }
      return value;
    }

    float computeSample(const vec3f &samplePos)
    {
#if 1
      return computeSampleRec(samplePos,0);
#else
      float sum = 0.f;

      for (size_t i=0;i<numParticles;i++) {
        float dist = distance(samplePos,particle[i]);
        sum += splatValue(dist);
      }
      return sum;
#endif
    }
  };

  void buildBlock(Splatter &splatter, 
                  MappedVolume3f &mappedVol, 
                  Block &block)
  {
    for (size_t z=0;z<block.dims.z;z++)
      for (size_t y=0;y<block.dims.y;y++)
        for (size_t x=0;x<block.dims.x;x++) {
          const vec3i cell(block.lower.x+x,block.lower.y+y,block.lower.z+z);
          const vec3f pos = splatter.getWorldPos(cell,mappedVol.dims);
          block.voxel[z][y][x] = splatter.computeSample(pos);
        }
    mappedVol.writeBlock(block);
  }

  void pkd2volume(int ac, char **av)
  {
    std::string inFileName;
    std::string outFileName;
    vec3i dims(0);
    Splatter splatter;
    splatter.radius = 1.f;
    splatter.border = .5f;

    for (int i=1;i<ac;i++) {
      std::string arg = av[i];
      if (arg[0] == '-') {
        if (arg == "-o") {
          assert(i+1 < ac);
          outFileName = av[++i];
        } else if (arg == "--border" || arg == "-b") {
          assert(i+1 < ac);
          splatter.border = atof(av[++i]);
        } else if (arg == "--radius" || arg == "-r") {
          assert(i+1 < ac);
          splatter.radius = atof(av[++i]);
        } else if (arg == "-dims") {
          assert(i+3 < ac);
          dims.x = atoi(av[++i]);
          dims.y = atoi(av[++i]);
          dims.z = atoi(av[++i]);
        } else 
          usage("unkown parameter '"+arg+"'");
      } else {
        if (inFileName != "")
          usage("input already specified");
        inFileName = arg;
      }
    }
    if (inFileName == "")
      usage("no input specified");
    if (outFileName == "")
      usage("no output specified");
    if (dims.x < 1 || dims.y < 1 || dims.z < 1)
      usage("no valid dimensions specified");
    
    Ref<sg::World> world = sg::loadOSP(inFileName);
    if (!world)
      throw std::runtime_error("could not load input '"+inFileName+"'");

    if (world->node.size() != 1)
      throw std::runtime_error("input file not in expected pkd format (1)'");
    
    Ref<sg::PKDGeometry> pkd = world->node[0].dynamicCast<sg::PKDGeometry>();
    if (!pkd)
      throw std::runtime_error("input file not in expected pkd format (2)'");

    // =======================================================
    // do the actual work
    // =======================================================

    MappedVolume3f mappedVol(dims, outFileName+"bin");
    
    splatter.particle = pkd->particle3f;
    splatter.numParticles = pkd->numParticles;
    splatter.bounds = pkd->getBounds();
    vec3i blockID;
#pragma omp parallel
    for (blockID.z = 0; blockID.z*BLOCK_SIZE<dims.z; blockID.z++)
#pragma omp parallel
      for (blockID.y = 0; blockID.y*BLOCK_SIZE<dims.y; blockID.y++)
#pragma omp parallel
        for (blockID.x = 0; blockID.x*BLOCK_SIZE<dims.x; blockID.x++) {
          Block block(blockID,dims);
          buildBlock(splatter,mappedVol,block);
        }

    // =======================================================
    // done generating the bin file; let's write the osp file
    // =======================================================
    FILE *file = fopen(outFileName.c_str(),"w");

    fprintf(file,"<?xml?>\n");
    fprintf(file,"<ospray>\n");
    fprintf(file,"  <StructuredVolume voxelType=\"float\"\n");
    fprintf(file,"                    dimensions=\"%i %i %i\"\n",dims.x,dims.y,dims.z);
    fprintf(file,"                    ofs=\"0\"\n");
    fprintf(file,"                    />\n");
    fprintf(file,"</ospray>\n");
    fclose(file);
  }
}

int main(int ac, char **av)
{
  ospray::pkd2volume(ac,av);
  return 0;
}

