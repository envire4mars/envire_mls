
// TODO: should be set over config
#define ROBOT_NAME std::string("Asguard_v4")
// TODO: should be set over config
#define ROBOT_ROOT_LINK_NAME std::string("body")
// TODO: do we need this?
#define ASGUARD_PATH std::string("/models/robots/asguard_v4/smurf/asguard_v4.smurf")
// This is the name of the mls frame in the serialized graph that can be loaded
// by mars
#define DUMPED_MLS_FRAME_NAME std::string("mls_map")

//movingForward = false;
//bool sceneLoaded;
//bool moved;

//#define TEST_MLS_PATH std::string("/simulation/mars/plugins/envire_mls/testMlsData/crater_simulation_mls.graph")
//#define TEST_MLS_PATH std::string("/simulation/mars/plugins/envire_mls/testMlsData/mls_map-cave-20171109.graph")
#define TEST_MLS_PATH std::string("/simulation/mars/plugins/envire_mls/testMlsData/mls_map-cave-20171110.graph")

const std::vector<std::string> MOTOR_NAMES{"wheel_front_left_motor", "wheel_front_right_motor", "wheel_rear_left_motor", "wheel_rear_right_motor"};

const bool MOVE_FORWARD=true;
const double SPEED=0.5;

const bool LOAD_PLY=true;

const std::string PLY_FILE = "/simulation/mars/plugins/envire_mls/testPLYData/pointcloud-20171110-2230.ply";

/* HEADER
    void testAddMLS();
    void testAddMLSAndRobot();
*/

namespace mars {
  namespace plugins {
    namespace envire_mls {

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
    }
  }
}
