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

#include <base/TransformWithCovariance.hpp>
#include <base/samples/RigidBodyState.hpp>

#include <mars/sim/NodePhysics.h>
#include <mars/interfaces/sim/LoadCenter.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/plugins/envire_managers/EnvireDefs.hpp>
#include <mars/plugins/envire_managers/EnvireStorageManager.hpp>
#include <mars/sim/SimMotor.h>

#include <pcl/io/ply_io.h>

#include "defs.hpp"

namespace mars {
  namespace plugins {
    namespace envire_mls {

      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace mars::plugins;
      using namespace envire::core;
      using namespace maps::grid;


      EnvireMls::EnvireMls(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "EnvireMls") 
      {
        LOG_INFO("[EnvireMls::EnvireMls] Plugin instantiated");
        mlsFrameId = MLS_FRAME_NAME; 
        centerFrameId = SIM_CENTER_FRAME_NAME;
        ground_cfm = 0.00000001;
        ground_erp = 0.1;
        mlsLoaded = false; // Loaded in the graph and also as attribute of this class
      }

      void EnvireMls::init() 
      {
        #ifndef DEBUG_ENVIRE_MLS
          LOG_DEBUG( 
            "[EnvireMls::init] Debugging traces for this plugin are deactivated."
            "To activate them change the defines.");
        #endif
        conditionalDebugMsg("[EnvireMls::init] Envire_mls plugin is initializing");        
        envire::core::FrameId center = SIM_CENTER_FRAME_NAME; 
        std::shared_ptr<envire::core::EnvireGraph> simGraph = envire_managers::EnvireStorageManager::instance()->getGraph();
        if (! simGraph->containsFrame(center))
        {
          simGraph->addFrame(center);
        }

      }

      void EnvireMls::reset() { }

      EnvireMls::~EnvireMls() { }

      void EnvireMls::update(sReal time_ms) 
      {
        LOG_ERROR("[EnvireMls::Update] Implementation pending");

      }

      mlsPrec EnvireMls::getMLSFromFrame(const std::shared_ptr<envire::core::EnvireGraph> & graph, envire::core::FrameId frameId)
      {
        /*
        Extract from the graph the mls stored in frameId. It is assumed that
        the mls is stored in precalculated format 
        */
        EnvireGraph::ItemIterator<Item<mlsPrec>> beginItem, endItem;
        boost::tie(beginItem, endItem) = graph->getItems<Item<mlsPrec>>(frameId);
        if (beginItem != endItem)
        {
          mls = beginItem->getData(); 
          mlsLoaded = true;
          conditionalDebugMsg("[EnvireMls::getMLSFromFrame] MLS loaded in envireMLS.");
        }
        else
        {
          conditionalDebugMsg("[EnvireMls::getMLSFromFrame] No MLS could be loaded.");
        }
        return mls;
      }

      void EnvireMls::preStepChecks(void)
      { 
        // Check that we have the collision frames 
        // TODO: Should be updated if more collidables are updated in the graph
        if(colFrames.empty()){
          getAllColFrames();
        }

        std::shared_ptr<envire::core::EnvireGraph> simGraph = envire_managers::EnvireStorageManager::instance()->getGraph();
        if(simGraph->containsFrame(mlsFrameId) && (!mlsLoaded))
        {
          mls = getMLSFromFrame(simGraph, mlsFrameId);
        }
      }

      /** 
      *
      * \brief Updates all frames that contain collidable objects 
      *
      */
      void EnvireMls::getAllColFrames(void)
      {
        // Find out the frames which contain a collidable put them in a vector
        envire::core::EnvireGraph::vertex_iterator  it, end;
        std::shared_ptr<envire::core::EnvireGraph> simGraph = envire_managers::EnvireStorageManager::instance()->getGraph();

        std::tie(it, end) = simGraph->getVertices(); // This is triggering a segfault if you set simGraph as an attribute of this module instead of accessing it on each method from the other module
        for(; it != end; ++it)
        {
            // See if the vertex has collision objects
            IterCollItem itCols, endCols;
            std::tie(itCols, endCols) = simGraph->getItems<CollisionItem>(*it);
            if(itCols != endCols)
            {
                envire::core::FrameId colFrame = simGraph->getFrameId(*it);
                colFrames.push_back(colFrame);
                conditionalDebugMsg("[EnvireMLS::getAllColFrames] Collision items found in frame " + colFrame);
            }
        }
      }

      /**
       * \brief This function creates the matrix where the contact points are stored
       *
       * pre:
       *      - number of contacts and positions known
       * post:
       *      - structure which can be used for the generation of the feedback joints
       */
      void EnvireMls::initContactParams(
        std::shared_ptr<std::vector<dContact>> & contactsPtr, 
        const smurf::ContactParams contactParams, int numContacts)
      {
        //MLS Has currently no contact parameters, we will use just the ones of the collidable by now
        //std::vector<dContact> * contacts = contactsPtr.get();
        contactsPtr->reserve(numContacts);
        contactsPtr->push_back(dContact());
        contactsPtr->operator[](0).surface.mode = dContactSoftERP | dContactSoftCFM; 
        //contactsPtr[0].surface.soft_cfm = contactParams.cfm;
        contactsPtr->operator[](0).surface.soft_cfm = ground_cfm;
        //std::cout << "[EnvireMls::InitContactParameters] contactsPtr[0].surface.soft_cfm " << contactsPtr[0].surface.soft_cfm << std::endl;
        //std::cout << "[EnvireMls::InitContactParameters] ContactParams.cfm : " << contactParams.cfm <<std::endl;
        //std::cout << "[EnvireMls::InitContactParameters] ContactParams.erp : " << contactParams.erp <<std::endl;
        //std::cout << "[EnvireMls::InitContactParameters] ContactParams.friction1 : " << contactParams.friction1 <<std::endl;
        //std::cout << "[EnvireMls::InitContactParameters] ContactParams.friction1 : " << contactParams.friction_direction1 <<std::endl;
        //contactsPtr[0].surface.soft_erp = contactParams.erp;
        contactsPtr->operator[](0).surface.soft_erp = ground_erp;
        if(contactParams.approx_pyramid) 
        {
          contactsPtr->operator[](0).surface.mode |= dContactApprox1;
        }                              
        contactsPtr->operator[](0).surface.mu = contactParams.friction1;
        contactsPtr->operator[](0).surface.mu2 = contactParams.friction2;
        if(contactsPtr->operator[](0).surface.mu != contactsPtr->operator[](0).surface.mu2)
          contactsPtr->operator[](0).surface.mode |= dContactMu2;

        // Move handleFrictionDirection to another method
        // check if we have to calculate friction direction1
        if(contactParams.friction_direction1){
          //std::cout << "[EnvireMls::initiContactParams] About to set friction direction" << std::endl;
          dVector3 v1;
          contactsPtr->operator[](0).surface.mode |= dContactFDir1;
          /*
           * Don't know how to do this part yet
           * TODO Improve based on what is Done in NearCallback 
           *
           * NOTE This if seems not to be entered anyway in the interactions with
           * the MLS
           *
           */
          contactsPtr->operator[](0).fdir1[0] = v1[0];
          contactsPtr->operator[](0).fdir1[1] = v1[1];
          contactsPtr->operator[](0).fdir1[2] = v1[2];
        }
        // then check for fds
        if(contactParams.fds1){
          contactsPtr->operator[](0).surface.mode |= dContactSlip1;
          contactsPtr->operator[](0).surface.slip1 = contactParams.fds1;
        }
        if(contactParams.fds2){
          contactsPtr->operator[](0).surface.mode |= dContactSlip2;
          contactsPtr->operator[](0).surface.slip2 = contactParams.fds2;
        }
        // Then set bounce and bounce_vel
        if(contactParams.bounce){
          contactsPtr->operator[](0).surface.mode |= dContactBounce;
          contactsPtr->operator[](0).surface.bounce = contactParams.bounce;
          contactsPtr->operator[](0).surface.bounce_vel = contactParams.bounce_vel;
        }
        // Apply parametrization to all contacts.
        for (int i=1;i<numContacts;i++){
          contactsPtr->push_back(dContact());
          contactsPtr->operator[](i) = contactsPtr->operator[](0);
        }
      }

      /**
       * \brief This function puts in the matrix where the contact points are stored the contact points
       *
       * pre:
       *      - number of contacts and positions known
       *      - matrix to store them
       * post:
       *      - matrix with the contact points
       */
      void EnvireMls::dumpFCLResult(const fcl::CollisionResultf &result, std::shared_ptr<std::vector<dContact>> & contactsPtr)
      { 
        //std::cout << "[EnvireMls::dumpFCLResults] To Dump: " << std::endl;
        //envire::core::Transform tfColMls = control->graph->getTransform(frameId, MLS_FRAME_NAME); 
        std::shared_ptr<envire::core::EnvireGraph> simGraph = envire_managers::EnvireStorageManager::instance()->getGraph();
        envire::core::Transform tfSimMls = simGraph->getTransform(centerFrameId, mlsFrameId);
        fcl::Transform3f trafo = tfSimMls.transform.getTransform().cast<float>();
        #ifdef DEBUG_ENVIRE_MLS
          std::stringstream ss;
          ss << "[EnvireMls::dumpFCLResults]: Trafo \n" << trafo.matrix() << "\n";
          for(size_t i=0; i< result.numContacts(); ++i)
          {
            const auto & cont = result.getContact(i);
            //auto pos = (trafo*cont.pos).transpose();
            //std::vector<float> pos;
            //pos.push_back((trafo*cont.pos).transpose()[0]);
            //pos.push_back((trafo*cont.pos).transpose()[1]);
            //pos.push_back((trafo*cont.pos).transpose()[2]);
            ss << "[EnvireMls::dumpFCLResults]: Contact transpose " << (trafo * cont.pos).transpose() << "\n";
            ss << "[EnvireMls::dumpFCLResults]: Contact normal transpose (*trafo.linear) " << (trafo.linear() * cont.normal).transpose() << "\n";
            ss << "[EnvireMls::dumpFCLResults]: Contact normal transpose " << cont.normal.transpose() << "\n";
            ss << "[EnvireMls::dumpFCLResults]: Contact penetration depth " << cont.penetration_depth << "\n";
            //std::stringstream spos;
            //spos << "Pos: ";
            //spos << pos <<"\n";
            //conditionalDebugMsg("Pos: " + std::to_string(pos[0]) + ", " + std::to_string(pos[1]) +", " + std::to_string(pos[2]) );
          }
          LOG_DEBUG(ss.str().c_str());
        #endif
        for(size_t i=0; i< result.numContacts(); ++i)
        {
          const auto & cont = result.getContact(i);
          auto pos = (trafo*cont.pos).transpose();
          //std::stringstream spos;
          //spos << "Pos: ";
          //spos << pos <<"\n";
          //conditionalDebugMsg(spos.str());
          //contactsPtr->operator[](i).geom.pos[0] = pos[0];
          //contactsPtr->operator[](i).geom.pos[1] = pos[1];
          //contactsPtr->operator[](i).geom.pos[2] = pos[2];
          contactsPtr->operator[](i).geom.pos[0] = (trafo*cont.pos).transpose()[0];
          contactsPtr->operator[](i).geom.pos[1] = (trafo*cont.pos).transpose()[1];
          contactsPtr->operator[](i).geom.pos[2] = (trafo*cont.pos).transpose()[2];
          //std::stringstream spos;
          //spos << "Pos: ";
          //spos << pos <<"\n";
          //conditionalDebugMsg(spos.str());
          const auto & normal = (trafo.linear()*cont.normal).transpose();
          if (normal.z() < 0.0)
          {
            Eigen::Vector3d::Map(contactsPtr->operator[](i).geom.normal) = -normal.cast<double>();
          }
          else
          {
            Eigen::Vector3d::Map(contactsPtr->operator[](i).geom.normal) = normal.cast<double>();
          }
          //contactsPtr[i].geom.normal[0] = normal[0];
          //contactsPtr[i].geom.normal[1] = normal[1];
          //contactsPtr[i].geom.normal[2] = normal[2];
          const auto &depth = cont.penetration_depth;
          contactsPtr->operator[](i).geom.depth = 2.0*std::abs(depth);
        }
        #ifdef DEBUG_ENVIRE_MLS
          std::cout << "[EnvireMls::dumpFCLResults] Result: " << std::endl;
          Vector vNormal;
          Vector contact_point;
          for(size_t i=0; i< result.numContacts(); ++i)
          {
            contact_point.x() = contactsPtr->operator[](i).geom.pos[0];
            contact_point.y() = contactsPtr->operator[](i).geom.pos[1];
            contact_point.z() = contactsPtr->operator[](i).geom.pos[2];
            vNormal[0] = contactsPtr->operator[](i).geom.normal[0];
            vNormal[1] = contactsPtr->operator[](i).geom.normal[1];
            vNormal[2] = contactsPtr->operator[](i).geom.normal[2];
            //const auto & cont = result.getContact(i);
            std::cout << "[EnvireMls::dumpFCLResults]: contactsPtr[i].geom.pos" << contact_point.transpose() << std::endl;
            std::cout << "[EnvireMls::dumpFCLResults]: contactsPtr[i].geom.normal " << vNormal.transpose() << std::endl;
            std::cout << "[EnvireMls::dumpFCLResults]: contactsPtr[i].geom.depth " << contactsPtr->operator[](i).geom.depth << std::endl;
          }
        #endif
      }

      /** 
       *
       * \brief Method called in computeMLSCollisions when collisions are found.
       * This method instantiates the correspondent contact joints.
       * The method is based on what nearCallback was doing
       */
      std::shared_ptr<std::vector<dContact>> EnvireMls::createContacts(
        const fcl::CollisionResultf & result, 
        smurf::Collidable collidable)
      {
        conditionalDebugMsg("[EnvireMls::CreateContacts] Collidable " + collidable.getName());
        // Init dContact
        std::shared_ptr<std::vector<dContact>> contactsPtr = std::make_shared<std::vector<dContact>>();
        const smurf::ContactParams contactParams = collidable.getContactParams();
        initContactParams(contactsPtr, contactParams, result.numContacts());
        dumpFCLResult(result, contactsPtr);
        return contactsPtr;
      }

      /** 
       *
       * \brief Called in step the world from the simulator. 
       * Computes the collisions points
       *
       * Go through all the nodes of the graph, for each one that has a
       * collision object or is an MLS compute the collisions to others.
       * Unless they have same group id.
       *
       */

      //void EnvireMls::computeMLSCollisions(void)
      //std::vector<mars::sim::ContactsPhysics> EnvireMls::getContactPoints(void)
      bool EnvireMls::updateContacts(void)
      {
        bool updated = false;
        contacts.clear();
        /// first check for collisions
        num_contacts = 0;
        log_contacts = false;
        create_contacts = true;
        std::shared_ptr<envire::core::EnvireGraph> simGraph = envire_managers::EnvireStorageManager::instance()->getGraph();
        // This if seems to be just for debugging
        if (simGraph->containsFrame(mlsFrameId))
        {
          envire::core::Transform tfMlsCen = simGraph->getTransform(mlsFrameId, centerFrameId);
          conditionalDebugMsg("[EnvireMls::getContactPoints]: Transformation between MLS and center " + tfMlsCen.toString());
        }
        //
        // Here only the collisions between the MLS and collidable objects are
        // processed
        // The frames that contain collidables are obtained in stepTheWorldChecks (only once)
        //
        int countCollisions = 0;
        for(unsigned int frameIndex = 0; frameIndex<colFrames.size(); ++frameIndex)
        {
          conditionalDebugMsg("[EnvireMls::getContactPoints]: Collision related to frame " + colFrames[frameIndex]);
          envire::core::Transform tfColCen = simGraph->getTransform(centerFrameId, colFrames[frameIndex]);
          //LOG_DEBUG("[WorldPhysics::computeMLSCollisions]: Transformation between sim center and robot colission frame %s", colFrames[frameIndex]);
          //LOG_DEBUG("[WorldPhysics::computeMLSCollisions]: %s", tfColCen.toString().c_str());
          // Transformation must be from the mls frame to the colision object frame
          envire::core::Transform tfMlsCol = simGraph->getTransform(mlsFrameId, colFrames[frameIndex]);
          fcl::Transform3f trafo = tfMlsCol.transform.getTransform().cast<float>(); 
          //LOG_DEBUG("[WorldPhysics::computeMLSCollisions]: About to provide transformation between MLS and %s", colFrames[frameIndex]); 
          //LOG_DEBUG("[WorldPhysics::computeMLSCollisions]: Transformation according to envire graph: %s", tfMlsCol.toString().c_str());
          std::stringstream ss;
          ss << trafo.matrix();
          conditionalDebugMsg("[EnvireMls::getContactPoints]: Trafo \n " + ss.str());
          // Get the collision objects -Assumes only one per frame-
          IterCollItem itCols;
          itCols = simGraph->getItem<CollisionItem>(colFrames[frameIndex]); 
          smurf::Collidable collidable = itCols->getData();
          urdf::Collision collision = collidable.getCollision();
          // Prepare fcl call
          fcl::CollisionRequestf request(10, true, 10, true);
          fcl::CollisionResultf result;
          bool collisionComputed = true;
          switch (collision.geometry->type){
            case urdf::Geometry::SPHERE:
              { //wheel_front_left_motor_col_wheel_front_left_03
                //std::cout << "Collision with a sphere" << std::endl;
                urdf::SphereSharedPtr sphereUrdf = urdf::dynamic_pointer_cast<urdf::Sphere>(collision.geometry);
                fcl::Spheref sphere(sphereUrdf->radius);
                conditionalDebugMsg("[EnvireMls::getContactPoints]: About to request fcl collision");
                fcl::collide_mls(mls, trafo, &sphere, request, result);
                conditionalDebugMsg("[EnvireMls::getContactPoints]: Request form fcl collision answered");
                break;
              }
            case urdf::Geometry::BOX:
              {
                //std::cout << "Collision with a box" << std::endl;
                urdf::BoxSharedPtr boxUrdf = urdf::dynamic_pointer_cast<urdf::Box>(collision.geometry);
                fcl::Boxf box(boxUrdf->dim.x, boxUrdf->dim.y, boxUrdf->dim.z);
                fcl::collide_mls(mls, trafo, &box, request, result);
                break;
              }
            default:
              LOG_INFO("[EnvireMls::getContactPoints]: Collision with the selected geometry type not implemented");
              collisionComputed = false;
          }
          if (collisionComputed)
          {
            bool isCollision = result.isCollision();
            if (result.isCollision())
            {
              conditionalDebugMsg("[EnvireMls::getContactPoints]: Collision Detected " + colFrames[frameIndex]);
              //std::cout << "\n [WorldPhysics::computeMLSCollisions]: Collision detected related to frame " << colFrames[frameIndex] << std::endl;
              // Here a method createContacts will put the joints that correspond
              // Get the interface to the simNodes at the index
              //std::vector<std::shared_ptr<interfaces::NodeInterface>> nodesIfsPtrs;
              //envire::core::EnvireGraph::ItemIterator<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>> begin, end;
              //boost::tie(begin, end) = simGraph->getItems<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>(colFrames[frameIndex]);
              //if (begin != end){
              //  std::shared_ptr<mars::sim::SimNode> nodePtr = begin->getData();
              //  nodesIfsPtrs.push_back(nodePtr->getInterface());
              //}

              mars::sim::ContactsPhysics contacts_col;
              contacts_col.contactsPtr = createContacts(result, collidable);//, nodesIfsPtrs);
              contacts_col.collidable = std::make_shared<smurf::Collidable>(std::move(collidable));
              contacts_col.numContacts = result.numContacts();

              contacts.push_back(contacts_col);

              #ifdef DEBUG_ENVIRE_MLS
                for(size_t i=0; i< result.numContacts(); ++i)
                {
                  countCollisions ++;
                  const auto & cont = result.getContact(i);
                  std::stringstream ss;
                  ss << trafo.matrix();
                  ss << "[EnvireMls::getContactPoints] Contact transpose " << cont.pos.transpose() << "\n";
                  ss << "[EnvireMls::getContactPoints] Contact normal transpose " << cont.normal.transpose() << "\n";
                  ss << "[EnvireMls::getContactPoints] Contact penetration depth " << cont.penetration_depth << "\n";
                  LOG_DEBUG(ss.str().c_str());
                }
              #endif
            }
          }
        }
        conditionalDebugMsg("[EnvireMls::getContactPoints] Total collisions found: " + std::to_string(countCollisions));
        updated = true;
        return updated;
      }

      void EnvireMls::getSomeData(void* data) 
      { // TODO: if the mls is not loaded, do nothing
        bool ok = updateContacts();
        std::vector<mars::sim::ContactsPhysics> * contactsCheck = 
          static_cast<std::vector<mars::sim::ContactsPhysics> *>(data);
        *contactsCheck = contacts; 
        conditionalDebugMsg(
          "[EnvireMls::getSomedata] Found contacts with " 
          + std::to_string(contactsCheck->size()) + " collidables.");
      }
        
      void EnvireMls::conditionalDebugMsg(const std::string trace)
      {
        #ifdef DEBUG_ENVIRE_MLS
          LOG_DEBUG(trace.c_str());
        #endif
      }

    } // end of namespace envire_mls
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::envire_mls::EnvireMls);
CREATE_LIB(mars::plugins::envire_mls::EnvireMls);
