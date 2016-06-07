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

#define SILENT

namespace ospray {
  namespace xyz {
    using std::cout;
    using std::endl;

    void importModelNoHeader(ParticleModel *model, const ospcommon::FileName &fileName)
    {
      int rc;
      FILE *file = fopen(fileName.c_str(),"r");
      if (!file) 
        throw std::runtime_error("could not open input file "+fileName.str());

      char line[10000]; 
      fgets(line,10000,file); // description line

      int i = 0;
      while (fgets(line,10000,file) && !feof(file)) {
        ++i;
        char atomName[110];
        vec3f p;
        vec3f n;
        rc = sscanf(line,"%100s %f %f %f %f %f %f\n",atomName,
                    &p.x,&p.y,&p.z,
                    &n.x,&n.y,&n.z
                    );
        if (rc != 7 && rc != 4) {
          std::stringstream ss;
          ss << "in " << fileName << " (line " << (i+2) << "): "
             << "could not parse .dat.xyz data line" << std::endl;
          throw std::runtime_error(ss.str());
        }
        int32 type = model->getAtomTypeID(atomName);
        model->type.push_back(type);
        model->position.push_back(p);
      }
    }


    void importModel(ParticleModel *model, const ospcommon::FileName &fileName)
    {
      FILE *file = fopen(fileName.c_str(),"r");
      if (!file) 
        throw std::runtime_error("could not open input file "+fileName.str());
      int numAtoms;

      // int rc = sscanf(line,"%i",&numAtoms);
      int rc = fscanf(file,"%i\n",&numAtoms);
      PRINT(numAtoms);
      if (rc != 1) {
        cout << "could not parse .dat.xyz header in input file " << fileName.str() << endl;
        cout << "trying to parse without header..." << endl;
        fclose(file);
        importModelNoHeader(model,fileName);
        return;
      }
      
      char line[10000]; 
      fgets(line,10000,file); // description line

      std::cout << "#" << fileName << " (.dat.xyz format): expecting " << numAtoms << " atoms" << std::endl;
      for (int i=0;i<numAtoms;i++) {
        char atomName[110];
        vec3f p;
        vec3f n;
        if (!fgets(line,10000,file)) {
          std::stringstream ss;
          ss << "in " << fileName << " (line " << (i+2) << "): "
             << "unexpected end of file!?" << std::endl;
          throw std::runtime_error(ss.str());
        }

        rc = sscanf(line,"%100s %f %f %f %f %f %f\n",atomName,
                    &p.x,&p.y,&p.z,
                    &n.x,&n.y,&n.z
                    );
        // rc = fscanf(file,"%100s %f %f %f %f %f %f\n",atomName,
        //             &a.position.x,&a.position.y,&a.position.z,
        //             &n.x,&n.y,&n.z
        //             );
        if (rc != 7 && rc != 4) {
          std::stringstream ss;
          PRINT(rc);
          PRINT(line);
          ss << "in " << fileName << " (line " << (i+2) << "): "
             << "could not parse .dat.xyz data line" << std::endl;
          throw std::runtime_error(ss.str());
        }
        int32 type = model->getAtomTypeID(atomName);
        model->type.push_back(type);
        model->position.push_back(p);
      }
    }

  } // ::ospray::particle
} // ::ospray

