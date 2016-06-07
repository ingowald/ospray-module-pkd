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

#pragma once

// ospray
#include "ospray/common/OSPCommon.h"
// embree
#include "common/constants.h"
#include "common/FileName.h"
// std
#include <map>
#include <vector>

#include "PKDConfig.h"
#include "ospray/MinMaxBVH2.h"

namespace ospray {
  
  /*! complete input data for a particle model */
  struct ParticleModel {
    typedef vec3f vec_t;

    ParticleModel() : radius(0) {}

    //! a set of attributes, one float per particle (with min/max info and logical name)
    struct Attribute {
      Attribute(const std::string &name) 
        : name(name), 
          minValue(+std::numeric_limits<float>::infinity()),
          maxValue(-std::numeric_limits<float>::infinity()) 
      {};

      std::string        name;
      float              minValue, maxValue;
      std::vector<float> value;
    };
    struct AtomType {
      std::string name;
      vec3f       color;

      AtomType(const std::string &name) : name(name), color(1,0,0) {}
    };
    
    //! list of all declared atom types
    std::vector<AtomType *>           atomType;
    //! mapper that maps an atom type name to the ID in the 'atomType' vector
    std::map<std::string, int32> atomTypeByName;

    uint32 getAtomTypeID(const std::string &name);

    std::vector<vec_t> position;   //!< particle position
    std::vector<int>   type;       //!< 'type' of particle (e.g., the atom type for atomistic models)
    std::vector<Attribute *> attribute;
#if PARTIKD_LIDAR_ENABLED
    box3f lidar_current_bounds = ospcommon::EmptyTy();
#endif

    //! \brief load a model (using the built-in model importers for
    //! various file formats). throw an exception if this cannot be
    //! done
    void load(const ospcommon::FileName &fn);

    //! get attributeset of given name; create a new one if not yet exists */
    Attribute *getAttribute(const std::string &name);

    //! return if attribute of this name exists 
    bool hasAttribute(const std::string &name);

    //! add one attribute value to set of attributes of given name
    void addAttribute(const std::string &attribName, float attribute);

    //! helper function for parser error recovery: 'clamp' all attributes to largest non-empty attribute
    void cullPartialData();

    //! return world bounding box of all particle *positions* (i.e., particles *ex* radius)
    box3f getBounds() const;

    float radius;  //!< radius to use (0 if not specified)
  };

}
