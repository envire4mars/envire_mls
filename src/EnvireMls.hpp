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
 * \file EnvireMls.h
 * \author Raul (Raul.Dominguez@dfki.de)
 * \brief Provides
 *
 * Version 0.1
 */

#pragma once

#include <ode/contact.h>
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/MARSDefs.h>
#include <mars/interfaces/NodeData.h>

#include <mars/sim/ContactsPhysics.hpp>
#include <string>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>

//#include <envire_collider_mls/MLSCollision.hpp>
#include <envire_fcl/Collision.hpp>

#include <maps/grid/MLSMap.hpp>


#include <mars/plugins/envire_smurf_loader/EnvireSmurfLoader.hpp>

namespace mars {

  namespace plugins {
    namespace envire_mls {

      using mlsPrec = maps::grid::MLSMapPrecalculated;
      using mlsKal = maps::grid::MLSMapKalman;
      using mlsSloped = maps::grid::MLSMapSloped;
      using mlsType = maps::grid::MLSMapPrecalculated;
      using CollisionType = smurf::Collidable;
      using CollisionItem = envire::core::Item<CollisionType>;
      using IterCollItem = envire::core::EnvireGraph::ItemIterator<CollisionItem>;

      // inherit from MarsPluginTemplateGUI for extending the gui
      class EnvireMls: public mars::interfaces::MarsPluginTemplate {

      public:
        EnvireMls(lib_manager::LibManager *theManager);
        ~EnvireMls();

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("envire_mls"); }
        CREATE_MODULE_INFO();

        // MarsPlugin methods
        void init();
        void reset();
        void update(mars::interfaces::sReal time_ms);
        void getSomeData(void* data);
        void preStepChecks(void);
        //void computeMLSCollisions(void);
        std::vector<mars::sim::ContactsPhysics> getContactPoints(void);
        void initContactParams(
          std::shared_ptr<dContact[]> contactPtr,
          const smurf::ContactParams contactParams, int numContacts);
        void dumpFCLResult(const fcl::CollisionResultf &result, std::shared_ptr<dContact[]> contactPtr);//, const envire::core::FrameId frameId);
        //void createFeedbackJoints( const envire::core::FrameId frameId, const smurf::ContactParams contactParams, dContact *contactPtr, int numContacts);
        
        std::shared_ptr<dContact[]> createContacts( 
          const fcl::CollisionResultf & result, 
          smurf::Collidable collidable, 
          const std::vector<std::shared_ptr<interfaces::NodeInterface>> & NodeIfsPtrsconst);

        // TODO: Consider moving this methods to another plugin, specialized on loading mls from .graph files
        void loadMLSMap(const std::string & mlsPath, const std::string & mls_frame_name);
        void addMLSNode();

        void getAllColFrames(void);



      private:

        //void deserializeMLS(const std::string & mlsPath);
        mars::interfaces::NodeData* setUpNodeData();
        mlsPrec getMLSFromFrame(const envire::core::EnvireGraph & graph, envire::core::FrameId mlsFrameId);
        void moveForwards();
        void loadSlopedFromPLY();

        // Private members
        maps::grid::MLSMapPrecalculated mlsPrecalculated;
        //envire::collision::MLSCollision *mlsCollision; // We might not need this 
        boost::shared_ptr<maps::grid::MLSMapPrecalculated> mlsPtr;

        // std::shared_ptr<envire::core::EnvireGraph> simGraph; // Getting some segfaults, maybe due to having this as attribute instead of getting it each time?

        envire::core::FrameId mlsFrameId;
        envire::core::FrameId centerFrameId;

        std::vector<envire::core::FrameId> colFrames;

        bool mlsLoaded;
        mlsType mls;

        std::vector<mars::sim::ContactsPhysics> contacts; 

        // Used in the computation of contacts/collisions
        // We keep same naming as in mars core
        bool create_contacts, log_contacts; 
        int num_contacts;
        mars::interfaces::sReal ground_cfm, ground_erp;


        //EnvireSmurfLoader::EnvireSmurfLoader* theLoader;

      }; // end of class definition EnvireMls

    } // end of namespace envire_mls
  } // end of namespace plugins
} // end of namespace mars

