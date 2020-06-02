#include "TestsPlugin.hpp"

#define TEST_MLS_PATH std::string("/models/environments/cueva_del_viento/mls_map-cave-20171110.graph")
// This is the name of the mls frame in the serialized graph that can be loaded
// by mars
#define DUMPED_MLS_FRAME_NAME std::string("mls_map")

namespace mars {
  namespace plugins {
    namespace envire_mls {

      void TestsPlugin::init() 
      {
        // Get the envire_mls_plugin so you can use it in the different tests
        LOG_DEBUG("Init envire mls plugin test");
        std::string dumpedGraphPath = std::getenv("AUTOPROJ_CURRENT_ROOT") + TEST_MLS_PATH;
        LOG_DEBUG( "Mls to test with: %s", dumpedGraphPath.c_str()); 
        // Get the envire mls plugin and load through it the map
        //EnvireMls* mlsPlugin = 
        //  libManager->acquireLibraryAs<EnvireMls>("envire_mls");
        //LOG_DEBUG("envire_mls library name: %s", mlsPlugin -> getLibName());
        //mlsPlugin -> loadMLSMap(dumpedGraphPath, DUMPED_MLS_FRAME_NAME);
      }
    }
  }
}
