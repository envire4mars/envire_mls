#include <string>
#include <vector>
#include <cstdlib>

const std::string testMlsDataPath = std::getenv("ENV_AUTOPROJ_ROOT") + std::string("/simulation/mars/plugins/envire_mls/tests/testMlsData/");
const std::string generalConfPath = testMlsDataPath + "general_conf.yml";
const std::vector<std::string> MOTOR_NAMES{"wheel_front_left_motor", "wheel_front_right_motor", "wheel_rear_left_motor", "wheel_rear_right_motor"};
const double SPEED=0.5;
const std::string dumpPlyTag = "dump_ply";
const std::string loadMlsGraphTag = "load_mls_graph";
const std::vector<std::string> confItems = {
  "robPos", "robOri",
  "mlsPos", "mlsOri",
  "robGoal" };
const std::string MLS_NAME = "CavePrecalculated";

