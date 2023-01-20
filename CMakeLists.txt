cmake_minimum_required(VERSION 3.9)
project(
  envire_mls
  VERSION 0.1
  DESCRIPTION "Provides MLS environment representation load and storage in the Envire Mars graph")

find_package(Rock)
set(ROCK_TEST_ENABLED ON CACHE BOOL "set to ON to enable the unit tests")

find_package(PCL 1.8 REQUIRED COMPONENTS common)
set(PCL_LIBS pcl_common-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR} pcl_io-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR})

find_package(PkgConfig REQUIRED)
pkg_check_modules(PKGCONFIG REQUIRED
                  lib_manager
                  envire_core
                  mars_sim
                  #envire_smurf_loader
                  envire_fcl
                  ${PCL_LIBS}
                  base-types
                  ode
)

rock_init()
rock_standard_layout()