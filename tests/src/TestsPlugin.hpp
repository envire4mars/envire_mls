#pragma once
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/plugins/envire_mls/EnvireMls.hpp>


namespace mars {
  namespace plugins {
    namespace envire_mls {

      class TestsPlugin: public mars::interfaces::MarsPluginTemplate {

      public:
        TestsPlugin(lib_manager::LibManager *theManager);
        ~TestsPlugin();

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("envire_mls_tests"); }
        CREATE_MODULE_INFO();

        // MarsPlugin methods
        void init();
        void reset();
        void update(mars::interfaces::sReal time_ms);

      private:
        EnvireMls* mlsPlugin;


      }; // end of class definition TestsPlugin

    } // end of namespace envire_mls
  } // end of namespace plugins
} // end of namespace mars