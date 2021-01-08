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
        LOG_DEBUG("EnvireMls plugin instantiated");
        mlsFrameId = MLS_FRAME_NAME; 
        centerFrameId = SIM_CENTER_FRAME_NAME;
        ground_cfm = 0.00000001;
        ground_erp = 0.1;
        // I think we are not using this library at all anymore
        //mlsCollision = envire::collision::MLSCollision::getInstance();
      }

      void EnvireMls::init() 
      {
        LOG_DEBUG("Envire_mls plugin is initializing");
#ifndef SIM_CENTER_FRAME_NAME
        LOG_ERROR( "[EnvireMls::init] SIM_CENTER_FRAME_NAME is not defined "); 
#endif
#ifdef SIM_CENTER_FRAME_NAME
        LOG_DEBUG( "[EnvireMls::init] SIM_CENTER_FRAME_NAME is defined: %s", SIM_CENTER_FRAME_NAME.c_str()); 
#endif
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

      mlsPrec EnvireMls::getMLSFromFrame(const envire::core::EnvireGraph & graph, envire::core::FrameId frameId)
      {
        /*
        Extract from the graph the mls stored in frameId. It is assumed that
        the mls is stored in Kalman format and has to be converted to precalculated.
        */
        EnvireGraph::ItemIterator<Item<mlsKal>> beginItem, endItem;
        boost::tie(beginItem, endItem) = graph.getItems<Item<mlsKal>>(frameId);
        mlsKal mlsKal;
        mlsPrec mls;
        mls = beginItem->getData(); // Here the conversion to Precalculated occurs (mlsPerc <-> mlsKal)
        return mls;
      }

      void EnvireMls::preStepChecks(void)
      { 
        // Check that we have the collision frames 
        // TODO The collision frames
        // should be updated if more collidables are included
        if(colFrames.empty()){
          getAllColFrames();
        }

        std::shared_ptr<envire::core::EnvireGraph> simGraph = envire_managers::EnvireStorageManager::instance()->getGraph();
        if(simGraph->containsFrame(mlsFrameId) && (!mlsLoaded))
        {
          envire::core::EnvireGraph::ItemIterator<envire::core::Item<mlsType>> beginItem, endItem;
          boost::tie(beginItem, endItem) = simGraph->getItems<envire::core::Item<mlsType>>(mlsFrameId);
          if (beginItem != endItem)
          {
            LOG_DEBUG("[EnvireMls::preStepChecks]: An mls was found in the simulation graph");
            mls = beginItem->getData();
            mlsLoaded = true;
            LOG_DEBUG("[EnvireMls::preStepChecks]: Mls map was fetched from the graph");
          }
          else
          {
            LOG_DEBUG("[EnvireMls::preStepChecks]: No Mls map was not found yet in the graph");
          }
        }
      }

      void EnvireMls::addMLSNode()
      {
        // TODO for loading various MLSs.
        // If the frame where the MLS should be
        // stored does not exists, create it by now we assume that the frame to
        // add to is the default one for the mls, created in the init step
        NodeData* nodePtr = setUpNodeData();
        std::shared_ptr<envire::core::EnvireGraph> simGraph = envire_managers::EnvireStorageManager::instance()->getGraph();
        envire::core::Item<NodeData>::Ptr itemPtr(new envire::core::Item<NodeData>(*nodePtr));
        simGraph->addItemToFrame(mlsFrameId, itemPtr);        
      }    

      /** 
      *
      * \brief Auxiliar methof of computeMLSCollisions. 
      * Returns all frames that contain collidable objects 
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
                LOG_DEBUG("[EnvireMLS::getAllColFrames] Collision items found in frame %s", colFrame.c_str());
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
        std::shared_ptr<dContact[]> contactsPtr, 
        const smurf::ContactParams contactParams, int numContacts)
        {
        //MLS Has currently no contact parameters, we will use just the ones of the collidable by now
        contactsPtr[0].surface.mode = dContactSoftERP | dContactSoftCFM;
        //contactsPtr[0].surface.soft_cfm = contactParams.cfm;
        contactsPtr[0].surface.soft_cfm = ground_cfm;
        //std::cout << "[EnvireMls::InitContactParameters] contactsPtr[0].surface.soft_cfm " << contactsPtr[0].surface.soft_cfm << std::endl;
        //std::cout << "[EnvireMls::InitContactParameters] ContactParams.cfm : " << contactParams.cfm <<std::endl;
        //std::cout << "[EnvireMls::InitContactParameters] ContactParams.erp : " << contactParams.erp <<std::endl;
        //std::cout << "[EnvireMls::InitContactParameters] ContactParams.friction1 : " << contactParams.friction1 <<std::endl;
        //std::cout << "[EnvireMls::InitContactParameters] ContactParams.friction1 : " << contactParams.friction_direction1 <<std::endl;
        //contactsPtr[0].surface.soft_erp = contactParams.erp;
        contactsPtr[0].surface.soft_erp = ground_erp;
        if(contactParams.approx_pyramid) 
        {
          contactsPtr[0].surface.mode |= dContactApprox1;
        }                              
        contactsPtr[0].surface.mu = contactParams.friction1;
        contactsPtr[0].surface.mu2 = contactParams.friction2;
        if(contactsPtr[0].surface.mu != contactsPtr[0].surface.mu2)
          contactsPtr[0].surface.mode |= dContactMu2;

        // Move handleFrictionDirection to another method
        // check if we have to calculate friction direction1
        if(contactParams.friction_direction1){
          //std::cout << "[EnvireMls::initiContactParams] About to set friction direction" << std::endl;
          dVector3 v1;
          contactsPtr[0].surface.mode |= dContactFDir1;
          /*
           * Don't know how to do this part yet
           * TODO Improve based on what is Done in NearCallback 
           *
           * NOTE This if seems not to be entered anyway in the interactions with
           * the MLS
           *
           */
          contactsPtr[0].fdir1[0] = v1[0];
          contactsPtr[0].fdir1[1] = v1[1];
          contactsPtr[0].fdir1[2] = v1[2];
        }
        // then check for fds
        if(contactParams.fds1){
          contactsPtr[0].surface.mode |= dContactSlip1;
          contactsPtr[0].surface.slip1 = contactParams.fds1;
        }
        if(contactParams.fds2){
          contactsPtr[0].surface.mode |= dContactSlip2;
          contactsPtr[0].surface.slip2 = contactParams.fds2;
        }
        // Then set bounce and bounce_vel
        if(contactParams.bounce){
          contactsPtr[0].surface.mode |= dContactBounce;
          contactsPtr[0].surface.bounce = contactParams.bounce;
          contactsPtr[0].surface.bounce_vel = contactParams.bounce_vel;
        }
        // Apply parametrization to all contacts.
        for (int i=1;i<numContacts;i++){
          contactsPtr[i] = contactsPtr[0];
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
      void EnvireMls::dumpFCLResult(const fcl::CollisionResultf &result, std::shared_ptr<dContact[]> contactsPtr)//, const envire::core::FrameId frameId)
      { 
        //std::cout << "[EnvireMls::dumpFCLResults] To Dump: " << std::endl;
        //envire::core::Transform tfColMls = control->graph->getTransform(frameId, MLS_FRAME_NAME); 
        std::shared_ptr<envire::core::EnvireGraph> simGraph = envire_managers::EnvireStorageManager::instance()->getGraph();
        envire::core::Transform tfSimMls = simGraph->getTransform(centerFrameId, mlsFrameId);
        fcl::Transform3f trafo = tfSimMls.transform.getTransform().cast<float>();
        // for debug //std::cout << "[EnvireMls::dumpFCLResults]: Trafo \n" << trafo.matrix() << std::endl;
        // for debug for(size_t i=0; i< result.numContacts(); ++i)
        // for debug {
        // for debug     const auto & cont = result.getContact(i);
        // for debug     //std::cout << "[EnvireMls::dumpFCLResults]: Contact transpose " << (trafo * cont.pos).transpose() << std::endl;
        // for debug     //std::cout << "[EnvireMls::dumpFCLResults]: Contact normal transpose (*trafo.linear) " << (trafo.linear() * cont.normal).transpose() << std::endl;
        // for debug     //std::cout << "[EnvireMls::dumpFCLResults]: Contact normal transpose " << cont.normal.transpose() << std::endl;
        // for debug     //std::cout << "[EnvireMls::dumpFCLResults]: Contact penetration depth " << cont.penetration_depth << std::endl;
        // for debug }
        for(size_t i=0; i< result.numContacts(); ++i)
        {
          const auto & cont = result.getContact(i);
          const auto & pos = (trafo*cont.pos).transpose();
          contactsPtr[i].geom.pos[0] = pos[0];
          contactsPtr[i].geom.pos[1] = pos[1];
          contactsPtr[i].geom.pos[2] = pos[2];
          const auto & normal = (trafo.linear()*cont.normal).transpose();
          if (normal.z() < 0.0)
          {
            Eigen::Vector3d::Map(contactsPtr[i].geom.normal) = -normal.cast<double>();
          }
          else
          {
            Eigen::Vector3d::Map(contactsPtr[i].geom.normal) = normal.cast<double>();
          }
          //contactsPtr[i].geom.normal[0] = normal[0];
          //contactsPtr[i].geom.normal[1] = normal[1];
          //contactsPtr[i].geom.normal[2] = normal[2];
          const auto &depth = cont.penetration_depth;
          contactsPtr[i].geom.depth = 2.0*std::abs(depth);
        }

        // for debug //std::cout << "[WorldPhysics::dumpFCLResults] Result: " << std::endl;
        // for debug Vector vNormal;
        // for debug Vector contact_point;
        // for debug for(size_t i=0; i< result.numContacts(); ++i)
        // for debug {
        // for debug   contact_point.x() = contactsPtr[0].geom.pos[0];
        // for debug   contact_point.y() = contactsPtr[0].geom.pos[1];
        // for debug   contact_point.z() = contactsPtr[0].geom.pos[2];
        // for debug   vNormal[0] = contactsPtr[i].geom.normal[0];
        // for debug   vNormal[1] = contactsPtr[i].geom.normal[1];
        // for debug   vNormal[2] = contactsPtr[i].geom.normal[2];
        // for debug   const auto & cont = result.getContact(i);
        // for debug   //std::cout << "[WorldPhysics::dumpFCLResults]:  contactsPtr[i].geom.pos" << contact_point.transpose() << std::endl;
        // for debug   //std::cout << "[WorldPhysics::dumpFCLResults]: contactsPtr[i].geom.normal " << vNormal.transpose() << std::endl;
        // for debug   //std::cout << "[WorldPhysics::dumpFCLResults]: contactsPtr[i].geom.depth " << contactsPtr[i].geom.depth << std::endl;
        // for debug }
      }

      /*
      // Most likely this has to be done in the simulator
      void EnvireMls::createFeedbackJoints( const envire::core::FrameId frameId, const smurf::ContactParams contactParams, dContact *contactPtr, int numContacts){
#ifd  ef DEBUG_WORLD_PHYSICS
        //std::cout << "[WorldPhysics::createFeedbackJoints] " << frameId << std::endl;
#end  if
        //numContacts is the number of collisions detected by fcl between the robot and the mls
        //num_contacts is a global variable of Worldphysics to keep track of the existent feedback joints
        dVector3 v;
        //dMatrix3 R;
        dReal dot;
        num_contacts++;
        if(create_contacts){
#ifd  ef DRAW_MLS_CONTACTS
          // NOTE Comment out this, so we don't draw the contacts
          draw_item item;
          item.id = 0;
          item.type = DRAW_LINE;
          item.draw_state = DRAW_STATE_CREATE;
          item.point_size = 10;
          item.myColor.r = 1;
          item.myColor.g = 0;
          item.myColor.b = 0;
          item.myColor.a = 1;
          item.label = "";
          item.t_width = item.t_height = 0;
          item.texture = "";
          item.get_light = 0;
#end  if
          for(int i=0;i<numContacts;i++){
            if(contactParams.friction_direction1) {
              v[0] = contactPtr[i].geom.normal[0];
              v[1] = contactPtr[i].geom.normal[1];
              v[2] = contactPtr[i].geom.normal[2];
              dot = dDOT(v, contactPtr[i].fdir1);
              dOPEC(v, *=, dot);
              contactPtr[i].fdir1[0] -= v[0];
              contactPtr[i].fdir1[1] -= v[1];
              contactPtr[i].fdir1[2] -= v[2];
              dNormalize3(contactPtr[0].fdir1);
            }
            contactPtr[0].geom.depth += (contactParams.depth_correction);
            if(contactPtr[0].geom.depth < 0.0) contactPtr[0].geom.depth = 0.0;
#ifd  ef DRAW_MLS_CONTACTS
            item.start.x() = contactPtr[i].geom.pos[0];
            item.start.y() = contactPtr[i].geom.pos[1];
            item.start.z() = contactPtr[i].geom.pos[2];
            item.end.x() = contactPtr[i].geom.pos[0] + contactPtr[i].geom.normal[0];
            item.end.y() = contactPtr[i].geom.pos[1] + contactPtr[i].geom.normal[1];
            item.end.z() = contactPtr[i].geom.pos[2] + contactPtr[i].geom.normal[2];
            draw_intern.push_back(item);
#end  if
            dJointID c=dJointCreateContact(world,contactgroup,contactPtr+i);

            envire::core::EnvireGraph::ItemIterator<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>> begin, end;
            boost::tie(begin, end) = control->graph->getItems<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>(frameId);

            if (begin != end){
#ifd  ef DEBUG_WORLD_PHYSICS
              //std::cout << "[WorldPhysics::createFeedbackJoints] We have the simnode! " << std::endl;
#end  if            
              dJointFeedback *fb;
              fb = (dJointFeedback*)malloc(sizeof(dJointFeedback));
              dJointSetFeedback(c, fb);
              contact_feedback_list.push_back(fb);
#ifd  ef DEBUG_WORLD_PHYSICS
              Vector contact_point;
              contact_point.x() = contactPtr[0].geom.pos[0];
              contact_point.y() = contactPtr[0].geom.pos[1];
              contact_point.z() = contactPtr[0].geom.pos[2];
              //std::cout << "[WorldPhysics::createFeedbackJoints]: Contact point x" << contact_point.x() << std::endl;
              //std::cout << "[WorldPhysics::createFeedbackJoints]: Contact point y" << contact_point.y() << std::endl;
              //std::cout << "[WorldPhysics::createFeedbackJoints]: Contact point z" << contact_point.z() << std::endl;
#end  if
              std::shared_ptr<mars::sim::SimNode> nodePtr = begin->getData();
              interfaces::NodeInterface * nodeIfPtr = nodePtr->getInterface();
              nodeIfPtr -> addContacts(c, numContacts, contactPtr[i], fb);
            }
          } // for numContacts
        } // if create contacts
#ifd  ef DEBUG_WORLD_PHYSICS
        //std::cout << "[WorldPhysics::createFeedbackJoints] All done here " << std::endl;
#end  if            
      }
      */





      /** 
       *
       * \brief Method called in computeMLSCollisions when collisions are found.
       * This method instantiates the correspondent contact joints.
       * The method is based on what nearCallback was doing
       */
      //void EnvireMls::createContacts(const fcl::CollisionResultf & result, smurf::Collidable collidable, const envire::core::FrameId frameId){
      std::shared_ptr<dContact[]> EnvireMls::createContacts(
        const fcl::CollisionResultf & result, 
        smurf::Collidable collidable, 
        const std::vector<std::shared_ptr<interfaces::NodeInterface>> & NodeIfsPtrs)
      {
        LOG_DEBUG("[EnvireMls::CreateContacts] Collidable %s", collidable.getName().c_str());
        // Init dContact
        //dContact *contactPtr = new dContact[result.numContacts()];
        std::shared_ptr<dContact[]> contactsPtr = std::shared_ptr<dContact[]>(new dContact[result.numContacts()]);
        const smurf::ContactParams contactParams = collidable.getContactParams();
        initContactParams(contactsPtr, contactParams, result.numContacts());
        dumpFCLResult(result, contactsPtr);//, frameId); // Pass here the frame id or the transformation to the object?
        return contactsPtr;
        // Here we have to copy the contact points to the contactPtr structure or
        // if not pass the result to createFeedbackJoints so that it uses them
        // In the final version the following to function is executed in StepTheWorld:
        // createFeedbackJoints(frameId, contactParams, contactPtr, result.numContacts());
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
      std::vector<mars::sim::ContactsPhysics> EnvireMls::getContactPoints(void)
      {
        std::vector<mars::sim::ContactsPhysics> res;
        /// first check for collisions
        num_contacts = 0;
        log_contacts = false;
        create_contacts = true;
        std::shared_ptr<envire::core::EnvireGraph> simGraph = envire_managers::EnvireStorageManager::instance()->getGraph();
        // This if seems to be just for debugging
        if (simGraph->containsFrame(mlsFrameId))
        {
          envire::core::Transform tfMlsCen = simGraph->getTransform(mlsFrameId, centerFrameId);
          LOG_DEBUG("[EnvireMls::computeMLSCollisions]: Transformation between MLS and center %s", tfMlsCen.toString().c_str());
        }
        //
        // Here only the collisions between the MLS and collidable objects are
        // processed
        // The frames that contain collidables are obtained in stepTheWorldChecks (only once)
        //
        int countCollisions = 0;
        for(unsigned int frameIndex = 0; frameIndex<colFrames.size(); ++frameIndex)
        {
          //LOG_DEBUG("[WorldPhysics::computeMLSCollisions]: Collision related to frame %s", colFrames[frameIndex]);
          envire::core::Transform tfColRobCen = simGraph->getTransform(centerFrameId, colFrames[frameIndex]);
          //LOG_DEBUG("[WorldPhysics::computeMLSCollisions]: Transformation between sim center and robot colission frame %s", colFrames[frameIndex]);
          //LOG_DEBUG("[WorldPhysics::computeMLSCollisions]: %s", tfColRobCen.toString().c_str());
          // Transformation must be from the mls frame to the colision object frame
          envire::core::Transform tfMlsCol = simGraph->getTransform(mlsFrameId, colFrames[frameIndex]);
          fcl::Transform3f trafo = tfMlsCol.transform.getTransform().cast<float>(); 
          //LOG_DEBUG("[WorldPhysics::computeMLSCollisions]: About to provide transformation between MLS and %s", colFrames[frameIndex]); 
          //LOG_DEBUG("[WorldPhysics::computeMLSCollisions]: Transformation according to envire graph: %s", tfMlsCol.toString().c_str());
          //std::cout << "[WorldPhysics::computeMLSCollisions]: Transformation in fcl format: " << trafo << std::endl;
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
                //LOG_DEBUG("[WorldPhysics::computeMLSCollisions]: About to request fcl collision");
                fcl::collide_mls(mls, trafo, &sphere, request, result);
                //LOG_DEBUG("[WorldPhysics::computeMLSCollisions]: Request form fcl collision answered");
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
              LOG_INFO("[EnvireMls::computeMLSCollisions]: Collision with the selected geometry type not implemented");
              collisionComputed = false;
          }
          if (collisionComputed)
          {
            //std::cout << "\n[WorldPhysics::computeMLSCollisions]: isCollision()==" << result.isCollision() << std::endl;
            if (result.isCollision())
            {
              //std::cout << "\n [WorldPhysics::computeMLSCollisions]: Collision detected related to frame " << colFrames[frameIndex] << std::endl;
              // Here a method createContacts will put the joints that correspond
              // Get the interface to the simNodes at the index
              std::vector<std::shared_ptr<interfaces::NodeInterface>> nodesIfsPtrs;
              envire::core::EnvireGraph::ItemIterator<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>> begin, end;
              boost::tie(begin, end) = simGraph->getItems<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>(colFrames[frameIndex]);
              if (begin != end){
                std::shared_ptr<mars::sim::SimNode> nodePtr = begin->getData();
                nodesIfsPtrs.push_back(nodePtr->getInterface());
              }

              mars::sim::ContactsPhysics contacts;
              contacts.contactPtrs = createContacts(result, collidable, nodesIfsPtrs);
              contacts.collidable = std::make_shared<smurf::Collidable>(std::move(collidable));
              contacts.numContacts = result.numContacts();

              res.push_back(contacts);

              // 2021.01.06 - TODO ADD THIS LINE: createContacts(result, collidable, colFrames[frameIndex] );  // Can I pass directly the node interface instead of the frameId?

              //debugging for(size_t i=0; i< result.numContacts(); ++i)
              //debugging {
              //debugging   countCollisions ++;
              //debugging   const auto & cont = result.getContact(i);
              //debugging   //std::cout << "[WorldPhysics::computeMLSCollisions]: Contact transpose " << cont.pos.transpose() << std::endl;
              //debugging   //std::cout << "[WorldPhysics::computeMLSCollisions]: Contact normal transpose " << cont.normal.transpose() << std::endl;
              //debugging   //std::cout << "[WorldPhysics::computeMLSCollisions]: Contact penetration depth " << cont.penetration_depth << std::endl;
              //debugging }
            }
          }
          return res;
        }
        LOG_DEBUG("Total collisions found: %i", countCollisions);
        //std::cout << "Total collisions found " << countCollisions << std::endl; 
        //std::cout << "Collision Check Finished " << std::endl;
      }



      // TODO: Consider moving this method to another plugin. This plugin should
      // be only for identifying the colisions between mls and other objects
      void EnvireMls::loadMLSMap(const std::string & mlsPath, const std::string & mls_frame_name)
      {
        /* Loads in the envire graph the mls given in the path after
         * deserializing it.
         *
         * The serialized object is graph containing the mls in DUMPED_MLS_FRAME
         */
        std::shared_ptr<envire::core::EnvireGraph> simGraph = envire_managers::EnvireStorageManager::instance()->getGraph();
        EnvireGraph auxMlsGraph;
        auxMlsGraph.loadFromFile(mlsPath);
        FrameId dumpedFrameId(mls_frame_name);
        mlsPrec mlsAux = getMLSFromFrame(auxMlsGraph, dumpedFrameId);
        Item<mlsPrec>::Ptr mlsItemPtr(new Item<mlsPrec>(mlsAux));
        simGraph->addItemToFrame(mlsFrameId, mlsItemPtr);
      }

      // Seems like it is not used at all for now and might not be needed, since the collision are not computed by ODE
      NodeData* EnvireMls::setUpNodeData()
      {
        /**
         * Look up the stored mls map and generate the correspondent MLSNodeData
         *
         * BUG: Currenty after one step the mls frame position is set to the
         * centre centerFrame.
         */

        std::shared_ptr<envire::core::EnvireGraph> simGraph = envire_managers::EnvireStorageManager::instance()->getGraph();
        mlsPrec mls = getMLSFromFrame(*(simGraph), mlsFrameId);
        Transform mlsTransform = simGraph->getTransform(centerFrameId, mlsFrameId);
#ifdef DEBUG
            LOG_DEBUG("[EnvireMls::addMLS] Tf x y z %f, %f, %f", 
                mlsTransform.transform.translation.x(), 
                mlsTransform.transform.translation.y(), 
                mlsTransform.transform.translation.z());
#endif
        Vector pos = mlsTransform.transform.translation;
        NodeData* node(new NodeData);
        //NodeData* node(new NodeData);
        node->init(mlsFrameId, pos);
    
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

      void EnvireMls::getSomeData(void* data) 
      {
        contacts = getContactPoints();
        std::vector<mars::sim::ContactsPhysics> * contactsCheck = 
          static_cast<std::vector<mars::sim::ContactsPhysics> *>(data);
        contactsCheck = &contacts;
        LOG_DEBUG("[EnvireMls::getSomedata] Found contacts with %i collidables.", 
                  contactsCheck->size());
      }

    } // end of namespace envire_mls
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::envire_mls::EnvireMls);
CREATE_LIB(mars::plugins::envire_mls::EnvireMls);
