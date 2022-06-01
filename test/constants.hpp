#include <string>
#include <vector>
#include <cstdlib>
#include <mars/plugins/envire_managers/EnvireDefs.hpp>

const std::string testMlsDataPath = std::getenv(ENV_AUTOPROJ_ROOT) + std::string("/simulation/envire_mls/test/testMlsData/");
const std::string asguardPath = std::getenv(ENV_AUTOPROJ_ROOT) + std::string("/models/robots/asguard_v4/smurf/asguard_v4.smurf");
const std::string robotName = "asguard";
const std::string generalConfPath = testMlsDataPath + "general_conf.yml";
const std::string sceneConfPath = testMlsDataPath + "scene_conf.yml";
const std::vector<std::string> MOTOR_NAMES{"wheel_front_left_motor", "wheel_front_right_motor", "wheel_rear_left_motor", "wheel_rear_right_motor"};
const double SPEED=1.0;
const std::string dumpPlyTag = "dump_ply";
const std::string loadMlsGraphTag = "load_mls_graph";
const std::vector<std::string> confItems = {
  "robPos", "robOri",
  "mlsPos", "mlsOri",
  "robGoal" };
const std::string MLS_NAME = "CavePrecalculated";

