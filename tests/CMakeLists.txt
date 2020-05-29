find_package(Boost REQUIRED COMPONENTS system)

rock_testsuite(sim_envire_mls_test
    suite.cpp
    test_sim_envire_mls.cpp
    DEPS_CMAKE Boost
    DEPS envire_mls)

target_link_libraries(sim_envire_mls_test ${Boost_LIBRARIES})