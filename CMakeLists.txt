project(envire_mls)
set(PROJECT_VERSION 1.0)
set(PROJECT_DESCRIPTION "Provides MLS environment representation load and storage in the Envire Mars graph")
cmake_minimum_required(VERSION 2.6)
include(FindPkgConfig)

find_package(lib_manager)
lib_defaults()
define_module_info()

find_package(PCL 1.8 REQUIRED COMPONENTS common)
set(PCL_LIBS pcl_common-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR} pcl_io-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR})

pkg_check_modules(PKGCONFIG REQUIRED
			    lib_manager
                            envire_core
                            mars_sim
                            envire_collider_mls
                            envire_smurf_loader
                            ${PCL_LIBS}
                            base-types
)
include_directories(${PKGCONFIG_INCLUDE_DIRS})
link_directories(${PKGCONFIG_LIBRARY_DIRS})
add_definitions(${PKGCONFIG_CFLAGS_OTHER})  #flags excluding the ones with -I

add_subdirectory(src)
add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME}
                      ${PKGCONFIG_LIBRARIES}
)

if(WIN32)
  set(LIB_INSTALL_DIR bin) # .dll are in PATH, like executables
else(WIN32)
  set(LIB_INSTALL_DIR lib)
endif(WIN32)


set(_INSTALL_DESTINATIONS
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION ${LIB_INSTALL_DIR}
	ARCHIVE DESTINATION lib
)


# Install the library into the lib folder
install(TARGETS ${PROJECT_NAME} ${_INSTALL_DESTINATIONS})

# Install headers into mars include directory
install(FILES ${HEADERS} DESTINATION include/mars/plugins/${PROJECT_NAME})

# Prepare and install necessary files to support finding of the library 
# using pkg-config
configure_file(${PROJECT_NAME}.pc.in ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)

find_package(Rock)
set(ROCK_USE_CXX11 TRUE)
rock_init(envire_mls 0.1)
rock_standard_layout()

