// TODO: should be set over config
#define MLS_FRAME_NAME std::string("mls_01")
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
//#define DEBUG_WORLD_PHYSICS 1 // Comment in to have logs from the physics simulator controller
#define DRAW_MLS_CONTACTS 1 // Comment in to have logs from the physics simulator controller