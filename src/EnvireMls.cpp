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
 * \file EnvireMls.cpp
 * \author Raul (Raul.Dominguez@dfki.de)
 * \brief Provides
 *
 * Version 0.1
 */

#include "EnvireMls.hpp"

#include <fstream>
#include <boost/archive/binary_iarchive.hpp>
#include <envire_core/items/Transform.hpp>


#include <envire_core/graph/GraphDrawing.hpp>

#include <mars/sim/NodePhysics.h>

#include <mars/interfaces/sim/LoadCenter.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <base/samples/RigidBodyState.hpp>
#include <mars/plugins/envire_managers/EnvireDefs.hpp>
#include <mars/plugins/envire_managers/EnvireStorageManager.hpp>
#include <mars/sim/SimMotor.h>

#include <base/TransformWithCovariance.hpp>

#include <pcl/io/ply_io.h>

//#define TEST_MLS_PATH std::string("/simulation/mars/plugins/envire_mls/testMlsData/crater_simulation_mls.graph")
//#define TEST_MLS_PATH std::string("/simulation/mars/plugins/envire_mls/testMlsData/mls_map-cave-20171109.graph")
#define TEST_MLS_PATH std::string("/simulation/mars/plugins/envire_mls/testMlsData/mls_map-cave-20171110.graph")
#define MLS_FRAME_TF_X 0.0
#define MLS_FRAME_TF_Y 0.0
#define MLS_FRAME_TF_Z 0.0
#define MLS_FRAME_TF_ROT_X 0.0 

#define GD_SENSE_CONTACT_FORCE 0
#define GD_PARENT_GEOM 0
#define GD_C_PARAMS_CFM 0.001
#define GD_C_PARAMS_ERP 0.001
#define GD_C_PARAMS_BOUNCE 0.0

#define ROBOT_TEST_POS  mars::utils::Vector(-3.5,-1,0)
#define ROBOT_TEST_Z_ROT  mars::utils::Vector(0,0,-90.0)

#define DEBUG 0


// TODO: should be set over config
#define MLS_FRAME_NAME std::string("mls_01")
// TODO: should be set over config
#define ROBOT_NAME std::string("Asguard_v4")
// TODO: should be set over config
#define ROBOT_ROOT_LINK_NAME std::string("body")
// TODO: do we need this?
#define ASGUARD_PATH std::string("/models/robots/asguard_v4/smurf/asguard_v4.smurf")
// This is the name of the mls frame in the serialized graph that can be loaded
// by mars
#define DUMPED_MLS_FRAME_NAME std::string("mls_map")
//#define DEBUG_WORLD_PHYSICS 1 // Comment in to have logs from the physics simulator controller
#define DRAW_MLS_CONTACTS 1 // Comment in to have logs from the physics simulator controller

#define ENV_AUTOPROJ_ROOT "AUTOPROJ_CURRENT_ROOT"




const std::vector<std::string> MOTOR_NAMES{"wheel_front_left_motor", "wheel_front_right_motor", "wheel_rear_left_motor", "wheel_rear_right_motor"};

const bool MOVE_FORWARD=true;
const double SPEED=0.5;

const bool LOAD_PLY=true;

const std::string PLY_FILE = "/simulation/mars/plugins/envire_mls/testPLYData/pointcloud-20171110-2230.ply";


namespace mars {
  namespace plugins {
    namespace envire_mls {

      using namespace mars::utils;
      using namespace mars::interfaces;

      using namespace envire::core;
      using namespace maps::grid;

      using namespace mars::plugins;


      EnvireMls::EnvireMls(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "EnvireMls") 
      {
        mlsCollision = envire::collision::MLSCollision::getInstance();
        //theLoader = theManager->getLibraryAs<EnvireSmurfLoader::EnvireSmurfLoader>("envire_smurf_loader");
      }

      void EnvireMls::init() 
      {
#ifdef DEBUG
        LOG_DEBUG( "[EnvireMls::init] Tests"); 
#endif
#ifndef SIM_CENTER_FRAME_NAME
        LOG_DEBUG( "[EnvireMls::init] SIM_CENTER_FRAME_NAME is not defined "); 
#endif
#ifdef SIM_CENTER_FRAME_NAME
        LOG_DEBUG( "[EnvireMls::init] SIM_CENTER_FRAME_NAME is defined: %s", SIM_CENTER_FRAME_NAME.c_str()); 
#endif
        envire::core::FrameId center = SIM_CENTER_FRAME_NAME; 
        std::shared_ptr<envire::core::EnvireGraph> graph = envire_managers::EnvireStorageManager::instance()->getGraph();
        if (! graph->containsFrame(center))
        {
          graph->addFrame(center);
        }
        // Create the default frame for the MLS but leave it empty.
        // The mls is loaded in the first update.
        mlsFrameId = MLS_FRAME_NAME; 
        centerFrameId = SIM_CENTER_FRAME_NAME;
        graph->addFrame(mlsFrameId);
        envire::core::Transform mlsTf(base::Time::now());
        mlsTf.transform.translation << double(MLS_FRAME_TF_X), double(MLS_FRAME_TF_Y), double(MLS_FRAME_TF_Z);
        mlsTf.transform.orientation = base::AngleAxisd(double(MLS_FRAME_TF_ROT_X), base::Vector3d::UnitX());
#ifdef DEBUG
        LOG_DEBUG("[EnvireMls::init] mlsTf x y z %f, %f, %f", 
            mlsTf.transform.translation.x(), 
            mlsTf.transform.translation.y(), 
            mlsTf.transform.translation.z());
        LOG_DEBUG("[EnvireMls::init] Constants for x y z %f, %f, %f", 
            double(MLS_FRAME_TF_X), 
            double(MLS_FRAME_TF_Y), 
            double(MLS_FRAME_TF_Z));
#endif
        graph->addTransform(SIM_CENTER_FRAME_NAME, MLS_FRAME_NAME, mlsTf);
        sceneLoaded = false;
        moved = false;
        movingForward = false;
      }

      void EnvireMls::reset() { }

      EnvireMls::~EnvireMls() { }

      void EnvireMls::update(sReal time_ms) 
      {
        LOG_ERROR("[EnvireMls::Update] Implementation pending");
        /*
        if (not sceneLoaded){
          //testAddMLS();
          testAddMLSAndRobot();

          sceneLoaded = true;
        }
        else{
            if (not moved){

                // Let's test the move robot method
                base::samples::RigidBodyState robotPose;
                robotPose.position << ROBOT_TEST_POS.x(), ROBOT_TEST_POS.y(), ROBOT_TEST_POS.z();
                robotPose.orientation = Eigen::AngleAxisd(ROBOT_TEST_Z_ROT.z(), Eigen::Vector3d::UnitZ());
                envire::core::Transform robotTf(robotPose.position, robotPose.orientation);
                //envire::core::Transform robotTf = robotPose.getTransform();
                LOG_DEBUG("[EnvireMls::update] Robot Target Pose: %g, %g, %g", robotTf.transform.translation.x(), robotTf.transform.translation.y(), robotTf.transform.translation.z());
                envire::core::FrameId robotRootFrame = ROBOT_ROOT_LINK_NAME;
                control->nodes->setTfToCenter(robotRootFrame, robotTf);
                //control->nodes->setTfToCenter(robotRootFrame, robotPose.getTransform());
                LOG_DEBUG("[EnvireMls::update] Robot moved");
                moved = true;
            }
            if ((MOVE_FORWARD) && (!movingForward)){
                moveForwards();    
            }
        }
        */
      }

      void EnvireMls::loadMLSMap(const std::string & mlsPath)
      {
        /* Loads in the envire graph the mls given in the path after
         * deserializing it.
         *
         * The serialized object is graph containing the mls in DUMPED_MLS_FRAME
         */
        EnvireGraph auxMlsGraph;
        auxMlsGraph.loadFromFile(mlsPath);
        FrameId dumpedFrameId(DUMPED_MLS_FRAME_NAME);
        mlsPrec mlsAux = getMLSMap(auxMlsGraph, dumpedFrameId);
        Item<mlsPrec>::Ptr mlsItemPtr(new Item<mlsPrec>(mlsAux));
        std::shared_ptr<envire::core::EnvireGraph> graph = envire_managers::EnvireStorageManager::instance()->getGraph();
        graph->addItemToFrame(mlsFrameId, mlsItemPtr);
      }

      mlsPrec EnvireMls::getMLSMap(const envire::core::EnvireGraph & graph, envire::core::FrameId frameId)
      {
        // The serialized Mls is not in Precalculated but in Kalman, so we have to convert it
        EnvireGraph::ItemIterator<Item<mlsKal>> beginItem, endItem;
        boost::tie(beginItem, endItem) = graph.getItems<Item<mlsKal>>(frameId);
        mlsKal mlsKal;
        mlsPrec mls;
        mls = beginItem->getData(); // Here the conversion to Precalculated occurs (mlsPerc <-> mlsKal)
        return mls;
      }

      NodeData* EnvireMls::setUpNodeData()
      {
        /**
         * Look up the stored mls map and generate the correspondent MLSNodeData
         *
         * BUG: Currenty after one step the mls frame position is set to the
         * centre centerFrame.
         */

        std::shared_ptr<envire::core::EnvireGraph> graph = envire_managers::EnvireStorageManager::instance()->getGraph();
        mlsPrec mls = getMLSMap(*(graph), mlsFrameId);
        Transform mlsTransform = graph->getTransform(centerFrameId, mlsFrameId);
#ifdef DEBUG
        LOG_DEBUG("[EnvireMls::addMLS] Tf x y z %f, %f, %f", 
            mlsTransform.transform.translation.x(), 
            mlsTransform.transform.translation.y(), 
            mlsTransform.transform.translation.z());
#endif
        Vector pos = mlsTransform.transform.translation;
        NodeData* node(new NodeData);
        //NodeData* node(new NodeData);
        node->init(MLS_FRAME_NAME, pos);
        
        LOG_DEBUG("EnvireMls: Missing definition of NODE_TYPE_MLS");
        //node->physicMode = interfaces::NODE_TYPE_MLS;

	boost::shared_ptr<maps::grid::MLSMapPrecalculated> mlsPtr(& mls);
        // Store MLS geometry in simulation nodeA
        // Do we have to do this? I think not...
        //node->g_mls = (void*)(mlsCollision->createNewCollisionObject(mlsPtr));//_userdata);	

        node->pos = mlsTransform.transform.translation; // The position was already set
        node->rot = mlsTransform.transform.orientation; // The position was already set

        // The position should be read from the envire graph

        //dVector3 pos; // = mlsTransform.transform.translation;
        //pos[ 0 ] = mlsTransform.transform.translation.x();
        //pos[ 1 ] = mlsTransform.transform.translation.y();
        //pos[ 2 ] = mlsTransform.transform.translation.z();

        // Rotate so Z is up, not Y (which is the default orientation)
        // NOTE is this to be done for all MLS or only for this particular case?
        dMatrix3 R;
        dRSetIdentity( R );
        //dRFromAxisAndAngle( R, 1, 0, 0, (3.141592/180) * 90 );  //DEGTORAD

        LOG_DEBUG("EnvireMls SetUp node position is not implemented yet");
        /*
        // Place it.
        dGeomSetRotation( (dGeomID)node->g_mls, R );
#ifdef DEBUG
        LOG_DEBUG("[EnvireMls::addMLS] Set Position to %f, %f, %f", pos[0], pos[1], pos[2]);
        LOG_DEBUG("[EnvireMls::addMLS] Tf x y z %f, %f, %f", 
            mlsTransform.transform.translation.x(), 
            mlsTransform.transform.translation.y(), 
            mlsTransform.transform.translation.z());
#endif
        dGeomSetPosition( (dGeomID)node->g_mls, pos[0], pos[1], pos[2]);

        // set geom data (move to its own method)
        mars::sim::geom_data* gd = new mars::sim::geom_data;
        (*gd).setZero();
        gd->sense_contact_force = GD_SENSE_CONTACT_FORCE;
        gd->parent_geom = GD_PARENT_GEOM;
        gd->c_params.cfm = GD_C_PARAMS_CFM;
        gd->c_params.erp = GD_C_PARAMS_ERP;
        gd->c_params.bounce = GD_C_PARAMS_BOUNCE;
        dGeomSetData((dGeomID)node->g_mls, gd);

        node->movable = false;	
        */
        return node;
      }


      void EnvireMls::addMLSNode()
      {
        // TODO for loading various MLSs.
        // If the frame where the MLS should be
        // stored does not exists, create it by now we assume that the frame to
        // add to is the default one for the mls, created in the init step
        NodeData* nodePtr = setUpNodeData();
        envire::core::Item<NodeData>::Ptr itemPtr(new envire::core::Item<NodeData>(*nodePtr));
        envire::core::FrameId mlsFrameId = MLS_FRAME_NAME;
        std::shared_ptr<envire::core::EnvireGraph> graph = envire_managers::EnvireStorageManager::instance()->getGraph();
        graph->addItemToFrame(mlsFrameId, itemPtr);        
      }

      void EnvireMls::testAddMLS()
      {
#ifdef DEBUG
        std::string path = std::getenv(ENV_AUTOPROJ_ROOT) + TEST_MLS_PATH;
        LOG_DEBUG( "[EnvireMls::addMLS] Mls to test with: %s", path.c_str()); 
#endif
        loadMLSMap(TEST_MLS_PATH);
        // Next is to instantiate a load the correspondent nodeData
        //addMLSNode();
        
#ifdef DEBUG
        LOG_DEBUG( "[EnvireMls::addMLS] 2"); 
#endif
      }

      void EnvireMls::testAddMLSAndRobot()
      {
#ifdef DEBUG
        std::string path = std::getenv(ENV_AUTOPROJ_ROOT) + TEST_MLS_PATH;
        LOG_DEBUG( "[EnvireMls::testAddMLSAndRobot] Mls to test with: %s", path.c_str()); 
#endif
        if (LOAD_PLY)
        {
            loadSlopedFromPLY();
        }
        else
        {
            loadMLSMap(std::getenv(ENV_AUTOPROJ_ROOT) + TEST_MLS_PATH);
        }
        // Next is to instantiate a load the correspondent nodeData
        //addMLSNode();
        
#ifdef DEBUG
        LOG_DEBUG( "[EnvireMls::testAddMLSAndRobot] 2"); 
#endif
        LOG_ERROR( "[EnvireMls::testAddMLSAndRobot] Missing LoadScene method not included yet"); 
        //control->sim->loadScene(std::getenv(ENV_AUTOPROJ_ROOT) + ASGUARD_PATH, ROBOT_NAME, ROBOT_TEST_POS, ROBOT_TEST_Z_ROT);

      }

      void EnvireMls::moveForwards()
      {
          mars::sim::SimMotor* motor;
          for(auto it: MOTOR_NAMES) {
            motor = control->motors->getSimMotorByName(it);
#ifdef DEBUG
            LOG_DEBUG( "[EnvireMls::moveForwards] Motor %s received", it.c_str()); 
#endif
            motor->setVelocity(SPEED);
#ifdef DEBUG
            LOG_DEBUG( "[EnvireMls::moveForwards] Motor %s set velocity sent", it.c_str()); 
#endif 
          }
          movingForward = true;
      }

      void EnvireMls::loadSlopedFromPLY()
      {
#ifdef DEBUG
          LOG_DEBUG( "[EnvireMls::loadSlopedFromPLY] Start"); 
#endif
          pcl::PLYReader reader;
          pcl::PointCloud<pcl::PointXYZ> cloud;
          reader.read(std::getenv(ENV_AUTOPROJ_ROOT) + PLY_FILE, cloud);
#ifdef DEBUG
          LOG_DEBUG( "[EnvireMls::loadSlopedFromPLY] Cloud has been generated"); 
#endif  
          Vector2ui num_cells {2500, 2500};
          const Vector2d resolution {0.3, 0.3};
          const MLSConfig config; // Thickness, gapsize
          mlsSloped mlsS(num_cells, resolution, config);
          mlsS.getLocalFrame().translation() << 0.5*mlsS.getSize(), 0;
          base::TransformWithCovariance tf = base::TransformWithCovariance::Identity();
          tf.setCovariance(base::TransformWithCovariance::Covariance::Identity()*0.001);
          mlsS.mergePointCloud(cloud, tf);
          Item<mlsPrec>::Ptr mlsItemPtr(new Item<mlsPrec>(mlsS));
          std::shared_ptr<envire::core::EnvireGraph> graph = envire_managers::EnvireStorageManager::instance()->getGraph();
          graph->addItemToFrame(mlsFrameId, mlsItemPtr);
#ifdef DEBUG
          LOG_DEBUG( "[EnvireMls::loadSlopedFromPLY] MLS loaded to graph"); 
#endif  
      }

    } // end of namespace envire_mls
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::envire_mls::EnvireMls);
CREATE_LIB(mars::plugins::envire_mls::EnvireMls);
