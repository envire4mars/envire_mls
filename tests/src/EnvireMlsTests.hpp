/*
 *  Copyright 2013, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the MARS simulation framework.
 *
 *  MARS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License
 *  as published by the Free Software Foundation, either version 3
 *  of the License, or (at your option) any later version.
 *
 *  MARS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with MARS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * \file EnvireMlsTests.h
 * \author Raul.Dominguez (Raul.Dominguez@dfki.de)
 * \brief Tests
 *
 * Version 0.1
 */

#ifndef MARS_PLUGINS_ENVIRE_MLS_TESTS_H
#define MARS_PLUGINS_ENVIRE_MLS_TESTS_H

#ifdef _PRINT_HEADER_
  #warning "EnvireMlsTests.h"
#endif

// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/MARSDefs.h>
#include <mars/data_broker/ReceiverInterface.h>
#include <mars/cfg_manager/CFGManagerInterface.h>

#include <string>
#include <yaml-cpp/yaml.h>

#include <mars/plugins/envire_mls/EnvireMls.hpp>

namespace mars {

  namespace plugins {
    namespace envire_mls_tests {

      // inherit from MarsPluginTemplateGUI for extending the gui
      class EnvireMlsTests: public mars::interfaces::MarsPluginTemplate{

      public:
        EnvireMlsTests(lib_manager::LibManager *theManager);
        ~EnvireMlsTests();

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

        // EnvireMlsTests methods
        bool loadMlsMap(); // Loads the mls for the test
        bool loadRobot(); //Loads the robot for the test
        bool loadScene(); //Loads the scene for the test

      private:
        bool yamlLoad(const std::string & confPath, YAML::Node & conf);
        bool loadSceneConf(const std::string & confPath);
        bool loadGeneralConf(const std::string & confPath);
        
        void moveForward();
        bool goalReached();

        cfg_manager::cfgPropertyStruct example;
        bool mlsLoaded;
        bool robotLoaded;
        bool robotMoving;

        envire::core::FrameId mlsFrameId;
        envire::core::FrameId centerFrameId;

        enum ConfParams { mlsPosP, 
                          mlsOriP,
                          robPosP, 
                          robOriP, 
                          robGoalP }; 

        std::map<std::string, ConfParams> mapStringParams;
        mars::utils::Vector robPos;
        double robOri;
        mars::utils::Vector mlsPos;
        double mlsOri;
        mars::utils::Vector robGoalPos;

        bool confLoaded;
        std::map<std::string, std::string> generalConf; //TODO: rename to conf

        bool sceneLoaded;

      }; // end of class definition EnvireMlsTests

    } // end of namespace envire_mls_tests
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_ENVIRE_MLS_TESTS_H
