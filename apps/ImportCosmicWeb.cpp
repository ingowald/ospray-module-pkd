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

#undef NDEBUG

#include "ospray/common/OSPCommon.h"
#include "apps/common/xml/XML.h"
#include "ParticleModel.h"
// embree
#include "common/sys/filename.h"

#define SILENT

namespace ospray {
  namespace cosmic_web {
    using std::cout;
    using std::endl;

    void importModel(ParticleModel *model, const embree::FileName &fileName)
    {
      FILE *file = fopen(fileName.c_str(),"r");
      if (!file) 
        throw std::runtime_error("could not open input file "+fileName.str());

      int np_local;
      float a,
        t,
        tau;
      int nts;
      float dt_f_acc,
        dt_pp_acc,
        dt_c_acc;
      int cur_checkpoint,
        cur_projection,
        cur_halofind;
      float massp;

      int blocksize=(32*1024*1024)/24;
      int num_writes;


      fread( &np_local, sizeof(int), 1, file );
      fread( &a, sizeof(float), 1, file );
      fread( &t, sizeof(float), 1, file );
      fread( &tau, sizeof(float), 1, file );
      fread( &nts, sizeof(int), 1, file );
      fread( &dt_f_acc, sizeof(float), 1, file );
      fread( &dt_pp_acc, sizeof(float), 1, file );
      fread( &dt_c_acc, sizeof(float), 1, file );
      fread( &cur_checkpoint, sizeof(int), 1, file );
      fread( &cur_projection, sizeof(int), 1, file );
      fread( &cur_halofind, sizeof(int), 1, file );
      fread( &massp, sizeof(float), 1, file );

      num_writes = np_local/blocksize + 1;

      // printf( "np_local: %d\n", np_local );
      // printf( "a: %f\n", a );
      // printf( "t: %f\n", t );
      // printf( "tau: %f\n", tau );
      // printf( "nts: %d\n", nts );
      // printf( "dt_f_acc: %f\n", dt_f_acc );
      // printf( "dt_pp_acc: %f\n", dt_pp_acc );
      // printf( "dt_c_acc: %f\n", dt_c_acc );
      // printf( "cur_checkpoint: %d\n", cur_checkpoint );
      // printf( "cur_projection: %d\n", cur_projection );
      // printf( "cur_halofind: %d\n", cur_halofind );
      // printf( "massp: %f\n", massp );
      // printf( "num_writes: %d\n", num_writes );

      
      while (1) {
        vec3f p,v;
        int rc;
        rc = fread(&p,sizeof(float),3,file);
        if (rc == 0) break;
        rc = fread(&v,sizeof(float),3,file);
        if (rc == 0) break;
        model->position.push_back(p);
        model->addAttribute("v",sqrtf(dot(v,v)));
      }
    }

  } // ::ospray::particle
} // ::ospray

