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
#include "ospray/common/Data.h"
#include "ospray/common/Model.h"
// ospcommon
#include "ospcommon/box.h"

namespace ospray {
  typedef box_t<float, 4> box4f;

  /*! defines a (binary) BVH with some float min/max value per
      node. The BVH itself does not specify the primitive type it is
      used with, or what those min/max values represent. */
  struct MinMaxBVH {
    struct PrimAbstraction {
      virtual size_t numPrims() = 0;
      // virtual size_t sizeOfPrim() = 0;
      // virtual void swapPrims(size_t aID, size_t bID) = 0;
      virtual box3f boundsOf(size_t primID) = 0;
      virtual float attributeOf(size_t primID) = 0;
    };

    /*! a node in a MinMaxBVH: a (4D-)bounding box, plus a child/leaf refence */
    struct Node : public box4f {
      uint64 childRef;
    };
    void buildRec(const size_t nodeID, 
                  PrimAbstraction *pa, const size_t begin, const size_t end);
    void initialBuild(PrimAbstraction *pa);
    void updateRanges(PrimAbstraction *pa);

    /*! to allow passing this pointer to ISCP: */
    const void *getNodePtr() const { assert(!node.empty()); return &node[0]; };
    /*! to allow passing this pointer to ISCP: */
    // const int64 *getItemListPtr() const { assert(!primID.empty()); return &primID[0]; };
    
    //  protected:
    /*! node vector */
    std::vector<Node> node;
    std::vector<uint32> primID;
    /*! node reference to the root node */
    uint64 rootRef;
    const box4f &getBounds() const { return node[0]; }
  };

  inline size_t maxDim(const vec3f &v) {
    const float maxVal = ospcommon::reduce_max(v);
    if (maxVal == v.x) {
      return 0;
    } else if (maxVal == v.y) {
      return 1;
    } else if (maxVal == v.z) {
      return 2;
    } else {
      assert(false && "Invalid max val index for vec!?");
      return -1;
    }
  }

}

