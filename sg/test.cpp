#include <memory>
#include "common/sg/importer/Importer.h"
#include "common/sg/geometry/Spheres.h"
#include "PKD.h"

using namespace ospray::sg;

struct Sphere {
  vec3f pos;
  float radius;

  Sphere() : pos(0), radius(1) {}
};

void tester(std::shared_ptr<Node> world, const FileName file) {
  std::cout << "Tester here, loading " << file << std::endl;

#if 1
  auto geom = createNode("pkd-test", "PKDGeometry");//->nodeAs<PKDGeometry>();
#else
  auto geom = createNode("pkd-test", "Spheres")->nodeAs<Spheres>();
  geom->createChild("bytes_per_sphere", "int", int(sizeof(Sphere)));
  geom->createChild("offset_center", "int", int(0));
  geom->createChild("offset_radius", "int", int(3*sizeof(float)));

  auto spheres = std::make_shared<DataVectorT<Sphere, OSP_RAW>>();
  spheres->setName("spheres");
  spheres->v = std::vector<Sphere>(1, Sphere());

  geom->add(spheres);

  auto materials = geom->child("materialList").nodeAs<MaterialList>();
  materials->item(0)["d"]  = 1.f;
  materials->item(0)["Kd"] = vec3f(1);
  materials->item(0)["Ks"] = vec3f(0.2f);

#endif
#if 1
  PING;
  auto model = createNode("test_model", "Model");
  model->add(geom);

  auto instance = createNode("test_instance", "Instance");
  instance->setChild("model", model);
  model->setParent(instance);

  world->add(model);
#endif
}

OSPSG_REGISTER_IMPORT_FUNCTION(tester, pkd);

