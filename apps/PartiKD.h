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

#include "ParticleModel.h"

namespace ospray {

  //! \brief particle-kd-tree class. 
  /*! \detailed Note that this class will actually re-order the
      particle model 'in place', so the order of the particles (and
      their attribute values etc) in the modle will change when this
      tree does its thing! */
  struct PartiKD {
    ParticleModel *model;
    size_t numParticles;
    size_t numInnerNodes;
    size_t numLevels;
    int quantizeOutput;
    int roundRobin;

    PartiKD(bool roundRobin=0,int quantizeOutput=0) 
      : model(NULL), numParticles(0), numInnerNodes(0), roundRobin(roundRobin), quantizeOutput(quantizeOutput)
    {};

    //! build particle tree over given model. WILL REORDER THE MODEL'S ELEMENTS
    void build(ParticleModel *model);
    
    //! save to xml+binary file
    void saveOSP(const std::string &fileName);
    //! save to xml+binary file(s)
    void saveOSP(FILE *xml, FILE *bin);

    /*! @{ \brief Balanced KD-tree helper functions */
    
    static __forceinline size_t leftChildOf(const size_t nodeID)  { return 2*nodeID+1; }
    static __forceinline size_t rightChildOf(const size_t nodeID) { return 2*nodeID+2; }
    static __forceinline size_t parentOf(const size_t nodeID)     { return (nodeID-1)/2; }
    __forceinline size_t isInnerNode(const size_t nodeID)   const { return nodeID < numInnerNodes; }
    __forceinline size_t isLeafNode(const size_t nodeID)    const { return nodeID >= numInnerNodes; }
    __forceinline size_t isValidNode(const size_t nodeID)   const { return nodeID < numParticles; }
    __forceinline size_t numInnerNodesOf(const size_t N)    const { return N/2; }
    __forceinline bool hasLeftChild(const size_t nodeID)    const { return isValidNode(leftChildOf(nodeID)); }
    __forceinline bool hasRightChild(const size_t nodeID)   const { return isValidNode(rightChildOf(nodeID)); }
    __forceinline static size_t isValidNode(const size_t nodeID, const size_t numParticles) { return nodeID < numParticles; }
    /*! @} */
    
    __forceinline float pos(const size_t nodeID, const size_t dim) const { return model->position[nodeID][dim]; }

    void buildRec(const size_t nodeID, const box3f &bounds, const size_t depth) const;

    //! helper function for building - swap two particles in the model
    inline void swap(const size_t a, const size_t b) const;

    // save the given particle's split dimension
    void setDim(size_t ID, int dim) const;
  };

}
