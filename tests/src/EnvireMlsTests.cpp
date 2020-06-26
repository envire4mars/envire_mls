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
 * \file EnvireMlsTests.cpp
 * \author Raul.Dominguez (Raul.Dominguez@dfki.de)
 * \brief Tests
 *
 * Version 0.1
 */


#include "EnvireMlsTests.hpp"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>

#include <base-logging/Logging.hpp>

#include "defs.hpp"

const std::vector<std::string> MOTOR_NAMES{"wheel_front_left_motor", "wheel_front_right_motor", "wheel_rear_left_motor", "wheel_rear_right_motor"};
const double SPEED=0.5;

namespace mars {
  namespace plugins {
    namespace envire_mls_tests {

      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace mars::plugins;

      EnvireMlsTests::EnvireMlsTests(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "EnvireMlsTests") {
          mlsLoaded = false;
          robotLoaded = false;
          robotMoving = false;
      }
  
      void EnvireMlsTests::init() {
        LOG_DEBUG("Initialization routine of the Envire MLS Tests simulation plugin");
        // Get the envire mls plugin and load through it the map
        mlsPlugin = libManager->acquireLibraryAs<envire_mls::EnvireMls>("envire_mls");
        LOG_DEBUG("%s library acquired", mlsPlugin -> getLibName().c_str());
      }

      void EnvireMlsTests::reset() {
      }

      EnvireMlsTests::~EnvireMlsTests() {
      }


      void EnvireMlsTests::update(sReal time_ms) {

        if (not mlsLoaded){
          LOG_DEBUG("Loading the MLS Map "); 
          std::string dumpedGraphPath = std::getenv("AUTOPROJ_CURRENT_ROOT") + TEST_MLS_PATH;
          LOG_DEBUG( "Mls to test with: %s", dumpedGraphPath.c_str()); 
          // FIXME: mlsPlugin -> loadMLSMap(dumpedGraphPath, DUMPED_MLS_FRAME_NAME);
          mlsLoaded = true;
          LOG_DEBUG("Loading of the MLS Map has been commanded"); 
        }
        if (not robotLoaded){
          LOG_DEBUG("Loading the robot "); 
          LOG_DEBUG("Loading of the robot has been commanded"); 
          loadRobot();
          robotLoaded = true;
        }
        if (not robotMoving){
          LOG_DEBUG("About to send control commands to the robot"); 
          cmdFwdDrive();
          robotMoving = true;
        }
      }

      void EnvireMlsTests::receiveData(const data_broker::DataInfo& info,
                                    const data_broker::DataPackage& package,
                                    int id) {
        // package.get("force1/x", force);
      }
  
      void EnvireMlsTests::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {

        if(_property.paramId == example.paramId) {
          example.dValue = _property.dValue;
        }
      }

      void EnvireMlsTests::loadRobot(){
        LOG_DEBUG("Loading robot")
        //control->sim->loadScene(std::getenv(ENV_AUTOPROJ_ROOT) + ASGUARD_PATH, ROBOT_NAME, ROBOT_TEST_POS, ROBOT_TEST_Z_ROT);
        LOG_DEBUG(
          "Setting the robot to < x, y, z, rot_z>: %g, %g, %g, %g",
          ROBOT_TEST_POS.x(), ROBOT_TEST_POS.y(), ROBOT_TEST_POS.z(), ROBOT_TEST_Z_ROT.z()
        );
        base::samples::RigidBodyState robotPose;
        robotPose.position << ROBOT_TEST_POS.x(), ROBOT_TEST_POS.y(), ROBOT_TEST_POS.z();
        robotPose.orientation = Eigen::AngleAxisd(ROBOT_TEST_Z_ROT.z(), Eigen::Vector3d::UnitZ());
        envire::core::Transform robotTf(robotPose.position, robotPose.orientation);
        //envire::core::Transform robotTf = robotPose.getTransform();
        LOG_DEBUG(
          "Robot target translation : %g, %g, %g", 
          robotTf.transform.translation.x(), 
          robotTf.transform.translation.y(), 
          robotTf.transform.translation.z());
        envire::core::FrameId robotRootFrame = ROBOT_ROOT_LINK_NAME;
        LOG_DEBUG("TODO: Implement position change in test");
        //control->nodes->setTfToCenter(robotRootFrame, robotTf);
        //control->nodes->setTfToCenter(robotRootFrame, robotPose.getTransform());
        //LOG_DEBUG("Robot moved");
      }

      void EnvireMlsTests::cmdFwdDrive()
      {
          mars::sim::SimMotor* motor;
          for(auto it: MOTOR_NAMES) {
            motor = control->motors->getSimMotorByName(it);
            LOG_DEBUG( "Motor %s received", it.c_str()); 
            //motor->setVelocity(SPEED);
            LOG_DEBUG( "TODO: Motor %s set velocity sent", it.c_str()); 
          }
          robotMoving = true;
      }


    } // end of namespace envire_mls_tests
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::envire_mls_tests::EnvireMlsTests);
CREATE_LIB(mars::plugins::envire_mls_tests::EnvireMlsTests);
