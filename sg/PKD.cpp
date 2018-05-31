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

#include <memory>
#include "PKD.h"
#include "ospcommon/xml/XML.h"
#include "common/sg/importer/Importer.h"
#include "common/sg/transferFunction/TransferFunction.h"
#include "common/sg/common/Common.h"
#include "ospcommon/xml/XML.h"
#include "ospcommon/constants.h"

namespace ospray {
  namespace sg {

    using std::string;
    using std::cout;
    using std::endl;

    //! constructor
    PKDGeometry::PKDGeometry() 
      : Geometry("pkd_geometry")
    {
      PING;
    }


    box3f PKDGeometry::bounds() const
    {

      box3f box = empty;
      if (hasChild("position")) {
        auto pos = child("position").nodeAs<DataBuffer>();
        if (pos->getType() == OSP_FLOAT3) {
          for (size_t i = 0; i < pos->size(); ++i) {
            box.extend(pos->get<vec3f>(i));
          }
        } else if (pos->getType() == OSP_ULONG) {
          for (size_t i = 0; i < pos->size(); ++i) {
            const uint64_t p = pos->get<uint64_t>(i);
            box.extend(decodeParticle(p));
          }
        }
      }
      if (hasChild("radius")) {
        const float radius = child("radius").valueAs<float>();
        box.lower -= vec3f(radius);
        box.upper += vec3f(radius);
      }
      return box;
    }

    vec3f PKDGeometry::decodeParticle(uint64_t i) const {
      const uint64_t mask = (1 << 20) - 1;
      const uint64_t ix = (i >> 2) & mask;
      const uint64_t iy = (i >> 22) & mask;
      const uint64_t iz = (i >> 42) & mask;
      return vec3f(ix, iy, iz);
    }

    void PKDGeometry::postCommit(RenderContext &)
    {
      auto geom = valueAs<OSPGeometry>();
      ospSetObject(geom, "transferFunction",
                   child("transferFunction").valueAs<OSPTransferFunction>());
      ospCommit(geom);
    }

    void importPKD(std::shared_ptr<Node> world, const ospcommon::FileName fileName)
    {
      std::cout << "Loading PKDGeometry from " << fileName << std::endl;

      // Read the radius from the XML file
      // Map the binary file and hand it off to the PKDGeometry node
      // as a shared data pointer.
      // We then also need to make a transfer function in the case that the
      // data has attributes
      auto doc = xml::readXML(fileName);
      const std::string binFileName = fileName.str() + "bin";
      unsigned char *binBasePtr = const_cast<unsigned char*>(mapFile(binFileName));
      if (!binBasePtr) {
        std::cout << "Failed to load corresponding pkdbin file for " << fileName.str() << "\n";
        throw std::runtime_error("Failed to load corresponding pkdbin file for "
                                  + fileName.str());
      }
      auto geom = createNode(fileName.str(), "PKDGeometry")->nodeAs<PKDGeometry>();

      const xml::Node &pkdNode = doc->child[0].child[0];
      if (pkdNode.name != "PKDGeometry") {
        std::cout << "failed to find PKDGeometry child node\n";
        throw std::runtime_error("failed to find PKDGeometry child node");
      }
      for (const xml::Node &e : pkdNode.child) {
        // TODO: How are attributes stored again? I forgot, need to fix
        // the converter and make some examples
        if (e.name == "position") {
          const std::string format = e.getProp("format");
          const size_t offset = std::stoull(e.getProp("ofs"));
          const size_t count = std::stoull(e.getProp("count"));
          if (format == "vec3f" || format == "float3") {
            std::cout << "Loading uncompressed PKD\n";
            auto posData = std::make_shared<DataArray3f>(reinterpret_cast<vec3f*>(binBasePtr + offset), count, false);
            posData->setName("position");
            geom->add(posData);
          } else if (format == "uint64") {
            std::cout << "Loading quantized PKD\n";
            auto posData = std::make_shared<DataArrayT<uint64_t, OSP_ULONG>>(
                reinterpret_cast<uint64_t*>(binBasePtr + offset), count, false);
            posData->setName("position");
            geom->add(posData);
          }
        } else if (e.name == "radius") {
          geom->createChild("radius", "float", std::stof(e.content));
        } else if (e.name == "attribute") {
          std::cout << "Got attribute" << e.getProp("name") << "\n";
          const std::string format = e.getProp("format");
          const size_t offset = std::stoull(e.getProp("ofs"));
          const size_t count = std::stoull(e.getProp("count"));
          if (format == "float") {
            auto attribData = std::make_shared<DataArray1f>(reinterpret_cast<float*>(binBasePtr + offset), count, false);
            attribData->setName("attribute");
            geom->add(attribData);
          } else {
            std::cout << "Unsupported attribute type: " << format << "\n";
          }
        }
      }
      if (geom->hasChild("attribute")) {
        auto tfn = createNode("transferFunction", "TransferFunction")->nodeAs<TransferFunction>();
        // Start with everything opaque in the data (show all particles)
        tfn->child("alpha").nodeAs<DataVector2f>()->v[0].y = 1;
        geom->add(tfn);
      }

      auto materials = geom->child("materialList").nodeAs<MaterialList>();
      materials->item(0)["d"]  = 1.f;
      materials->item(0)["Kd"] = vec3f(1.f);
      materials->item(0)["Ks"] = vec3f(0.2f);

      world->add(geom);
    }

    OSP_REGISTER_SG_NODE(PKDGeometry);

    OSPSG_REGISTER_IMPORT_FUNCTION(importPKD, pkd);

  } // ::ospray::sg
} // ::ospray

