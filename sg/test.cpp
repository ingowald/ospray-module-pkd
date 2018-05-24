#include "common/sg/importer/Importer.h"

using namespace ospray::sg;


void tester(std::shared_ptr<Node>, const FileName file) {
  std::cout << "Tester here, loading " << file << std::endl;
}

OSPSG_REGISTER_IMPORT_FUNCTION(tester, pkd);

