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
#include <mars/plugins/envire_managers/EnvireDefs.hpp>
#include <mars/plugins/envire_managers/EnvireStorageManager.hpp>
#include <mars/sim/SimMotor.h>

#include <base-logging/Logging.hpp>



#include <envire_mls/defs.hpp>
#include "defs.hpp"
#include "constants.hpp"


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
        confLoaded = false;
        sceneLoaded = false;

        //Needed? mlsCollision = envire::collision::MLSCollision::getInstance();
        robPos = {0.0, 0.0, 0.0};
        robOri = 0.0;
        mlsPos = {0.0, 0.0, 0.0};
        mlsOri = 0.0;
        mapStringParams["robPos"] = robPosP;
        mapStringParams["robOri"] = robOriP;
        mapStringParams["mlsPos"] = mlsPosP;
        mapStringParams["mlsOri"] = mlsOriP;
        mapStringParams["robGoal"] = robGoalP;

        mlsFrameId = MLS_FRAME_NAME;
      }
  
      void EnvireMlsTests::init() {
        if (! loadGeneralConf(generalConfPath))
        {
          LOG_ERROR("Problem loading the main conf %s", generalConfPath.c_str());
        }
        else{
          if (! loadSceneConf(sceneConfPath))
          {
            LOG_ERROR("Problem loading the scene conf %s", sceneConfPath.c_str());
          }
          else
          {
            sceneLoaded = loadScene();
          }
        }
        #ifndef SIM_CENTER_FRAME_NAME
          LOG_DEBUG( "[EnvireMlsTests::init] SIM_CENTER_FRAME_NAME is not defined "); 
        #endif
        #ifdef SIM_CENTER_FRAME_NAME
          LOG_DEBUG( "[EnvireMlsTests::init] SIM_CENTER_FRAME_NAME is defined: %s", SIM_CENTER_FRAME_NAME.c_str()); 
        #endif
        /*
        std::shared_ptr<envire::core::EnvireGraph> simGraph = envire_managers::EnvireStorageManager::instance()->getGraph();
        if (simGraph->containsFrame(MLS_FRAME_NAME))
        {
          envire::core::Transform tfMlsCen = simGraph->getTransform(MLS_FRAME_NAME, SIM_CENTER_FRAME_NAME);
          LOG_DEBUG("[EnvireMlsTests::Init]: Transformation between MLS and center %s", tfMlsCen.toString()); 
        }
        */
        control->sim->StartSimulation();
        LOG_INFO("[EnvireMlsTests::init] Simulation started for the test");
        
      }

      void EnvireMlsTests::reset() {
      }

      EnvireMlsTests::~EnvireMlsTests() {
      }


      void EnvireMlsTests::update(sReal time_ms) {
        if (sceneLoaded)
        {
          if (goalReached())
          {
            LOG_DEBUG( "[EnvireMlsTests::update] Goal was reached, simulation will be stopped"); 
            control->sim->StopSimulation();
          }
          else
          {
            if (!robotMoving)
            {
                moveForward();    
            }
          }
        }
      }


      bool EnvireMlsTests::loadMlsMap()
      {
        bool loaded = false;
        if (generalConf["mode"] == loadMlsGraphTag)
        {
            std::string testMlsPath = testMlsDataPath + generalConf["envire_graph_path"];
            Vector mlsRot(0,0,mlsOri);
            control->sim->loadScene(testMlsPath, MLS_NAME, mlsPos, mlsRot);
            loaded = true;
        }
        return loaded;
      }

      bool EnvireMlsTests::loadRobot(){
        bool loaded = false;
        LOG_DEBUG("Loading robot")
        utils::Vector robRot(0,0,robOri);
        control->sim->loadScene(asguardPath, robotName, robPos, robRot);
        LOG_DEBUG(
          "Setting the robot to < x, y, z, rot_z>: %g, %g, %g, %g",
          robPos[0], robPos[1], robPos[2], robOri
        );
        base::samples::RigidBodyState robotPose;
        robotPose.position << robPos[0], robPos[1], robPos[2];
        robotPose.orientation = Eigen::AngleAxisd(robOri, Eigen::Vector3d::UnitZ());
        envire::core::Transform robotTf(robotPose.position, robotPose.orientation);
        LOG_DEBUG(
          "Robot target translation : %g, %g, %g", 
          robotTf.transform.translation.x(), 
          robotTf.transform.translation.y(), 
          robotTf.transform.translation.z());
        envire::core::FrameId robotRootFrame = ROBOT_ROOT_LINK_NAME;
        LOG_DEBUG("TODO: Implement position change in test");
        loaded = true; // TODO check somehow
        return loaded;
      }

      bool EnvireMlsTests::loadScene()
      {
        if (loadMlsMap())
        {
          mlsLoaded = true;
          if (loadRobot())
          {
            robotLoaded = true;
            LOG_INFO("[EnvireMlsTests::loadScene] Scene loaded");
          }
          else
          {
            LOG_ERROR("[EnvireMlsTests::loadScene] Problem loading the robot");
          }
        }
        else
        {
            LOG_ERROR("[EnvireMlsTests::loadScene] Problem loading the mls");
        }
        return (robotLoaded && mlsLoaded);
      }

      void EnvireMlsTests::moveForward()
      {
          mars::sim::SimMotor* motor;
          for(auto it: MOTOR_NAMES) {
            motor = control->motors->getSimMotorByName(it);
            #ifdef FULL_MLS_TEST_DEBUG
              LOG_DEBUG( "[EnvireMlsTests::moveForwards] Motor %s received", it.c_str()); 
            #endif
            motor->setVelocity(SPEED);
            #ifdef FULL_MLS_TEST_DEBUG
              LOG_DEBUG( "[EnvireMlsTests::moveForwards] Motor %s set velocity sent", it.c_str()); 
            #endif
          }
          robotMoving = true;
      }

      bool EnvireMlsTests::yamlLoad(const std::string & confPath, YAML::Node & conf)
      {
        bool loaded = false;
        try{
          conf = YAML::LoadFile(confPath);
          loaded = true;
        }catch (...){
          LOG_ERROR("[EnvireMlsTests::init] Something went wrong loading the test config yaml."
          " It might have not been found. : %s", confPath.c_str());
        }
        return loaded;
      }


      bool EnvireMlsTests::loadGeneralConf(const std::string & confPath)
      {
        bool loaded = false;
        YAML::Node conf;
        if(yamlLoad(confPath, conf))
        {
          for (YAML::const_iterator it=conf.begin(); it!=conf.end();++it)
          {
            generalConf[it->first.as<std::string>()] = it->second.as<std::string>();
            LOG_DEBUG("[EnvireMlsTests::loadGeneralConf] %s : %s", it->first.as<std::string>().c_str(),  generalConf[it->first.as<std::string>()].c_str());
          }
          loaded = true;
        }
        return loaded;
      }

      bool EnvireMlsTests::loadSceneConf(const std::string & confPath)
      {
        bool loaded = false;
        YAML::Node conf;
        if(yamlLoad(confPath, conf))
        {
          for (const std::string& item: confItems)
          {
            LOG_INFO("[EnvireMlsTests::loadSceneConf] confItem: %s", item.c_str());
            if (conf[item])
            {
              switch (mapStringParams[item]){
                case robPosP:
                {
                  std::vector<double> robPosV = conf["robPos"].as<std::vector<double>>();
                  robPos = {robPosV[0], robPosV[1], robPosV[2]};
                  LOG_INFO(
                      "[EnvireMlsTests::loadSceneConf] Loaded this position for the rover: %f, %f, %f ",
                      robPos[0], robPos[1], robPos[2]);
                  break;

                }
                case robOriP:
                {
                  robOri = conf["robOri"].as<double>();
                  LOG_INFO(
                      "[EnvireMlsTests::loadSceneConf] Loaded orientation for the robot: %f ",
                      robOri);
                  break;
                }
                case mlsPosP:
                {
                  std::vector<double> mlsPosV = conf["mlsPos"].as<std::vector<double>>();
                  mlsPos = {mlsPosV[0], mlsPosV[1], mlsPosV[2]};
                  LOG_INFO(
                      "[EnvireMlsTests::loadSceneConf] Loaded this position for the mls: %f, %f, %f",
                      mlsPos[0], mlsPos[1], mlsPos[2]);
                  break;
                }
                case mlsOriP:
                {
                  mlsOri = conf["mlsOri"].as<double>();
                  LOG_INFO(
                      "[EnvireMlsTests::loadSceneConf] Loaded orientation for the mls: %f ",
                      mlsOri);
                  break;
                }
                case robGoalP:
                {
                  std::vector<double> robGoalV = conf["robGoal"].as<std::vector<double>>();
                  robGoalPos = {robGoalV[0], robGoalV[1], robGoalV[2]};
                  LOG_INFO(
                      "[EnvireMlsTests::loadSceneConf] Loaded goal position: %f, %f, %f ",
                      robGoalPos[0], robGoalPos[1], robGoalPos[2]);
                  break;
                }
              }
            }
          }
          loaded = true;
        }
        return loaded;
      }

      bool EnvireMlsTests::goalReached()
      {
        bool ok = true;
        bool reached = false;
        envire::core::Transform robPosTf;
        std::shared_ptr<envire::core::EnvireGraph> simGraph = envire_managers::EnvireStorageManager::instance()->getGraph();
        try{
          robPosTf = simGraph->getTransform(SIM_CENTER_FRAME_NAME, ROBOT_ROOT_LINK_NAME);
        }
        catch(const std::exception& e){
          LOG_ERROR(e.what());
          LOG_ERROR("Failed to find the robot in the envire graph. Does the frame %s exist in the graph?", ROBOT_ROOT_LINK_NAME.c_str());
          ok = false;
        }
        if (ok)
        {
          base::Position robPos = robPosTf.transform.translation;
          Eigen::Vector3f v;
          Eigen::Vector3f w;
          v << robPos[0], robPos[1], robPos[2];
          w << robGoalPos[0], robGoalPos[1], robGoalPos[2];
          Eigen::Vector3f diff = v - w;
          float distance = diff.norm();
          reached = (distance <= 0.2);
          if (reached)
          {
            LOG_INFO( "[EnvireMlsTests::goalReached] Target reached"); 
          }
          LOG_DEBUG( "[EnvireMlsTests::goalReached] Distance: %f", distance); 
        }
        return reached;
      }



    } // end of namespace envire_mls_tests
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::envire_mls_tests::EnvireMlsTests);
CREATE_LIB(mars::plugins::envire_mls_tests::EnvireMlsTests);
