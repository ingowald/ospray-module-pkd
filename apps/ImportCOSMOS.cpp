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
  namespace cosmos {
    using std::cout;
    using std::endl;

    void importModel(ParticleModel *model, const embree::FileName &fileName)
    {
      FILE *file = fopen(fileName.c_str(),"r");
      if (!file) 
        throw std::runtime_error("could not open input file "+fileName.str());
      std::cout << "#" << fileName << " (COSMOS format)" << std::endl;
      while (1) {
        float l1, l2, l3, data;
        int rc = fscanf(file,"%f %f %f %f\n",&l1,&l2,&l3,&data);
        if (rc != 4) break;
        
        float value =  ((2.0*l1+1.0)*(2.0*l2+1.0)*(2.0*l3+1.0))*((l1+l2+l3)*(l1+l2+l3+3))/(2*(l1+l2+l3)+3)*data;

        value *= 1e12f;

        // float min_val = -2e16;
        // float max_val = +2e16;

        // a = std::max(a,min_val);
        // a = std::min(a,max_val);

        model->position.push_back(vec3f(l1,l2,l3));
        model->addAttribute("COSMOS",value);
      }
    }

  } // ::ospray::particle
} // ::ospray

