set(SUBPROJECT_NAME envire_mls_tests)
include(FindPkgConfig)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PKGCONFIG REQUIRED
			    lib_manager
				yaml-cpp
			    mars_sim
                ${PCL_LIBS}
				envire_mls
				envire_core
)

include_directories(${PKGCONFIG_INCLUDE_DIRS})
link_directories(${PKGCONFIG_LIBRARY_DIRS})

set(SOURCES 
	EnvireMlsTests.cpp
)

set(HEADERS
	EnvireMlsTests.hpp
	defs.hpp
	constants.hpp
)

#rock_library  (${SUBPROJECT_NAME}
#        HEADERS ${HEADERS}
#        SOURCES ${SOURCES}
#        DEPS_PKGCONFIG ${deps_pkg_config})

message("Package headers " ${HEADERS} )

add_library(${SUBPROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(envire_mls_tests
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
install(TARGETS ${SUBPROJECT_NAME} ${_INSTALL_DESTINATIONS})

# Install headers into mars include directory
install(FILES ${HEADERS} DESTINATION include/mars/plugins/${SUBPROJECT_NAME})

# Prepare and install necessary files to support finding of the library 
# using pkg-config
configure_file(${SUBPROJECT_NAME}.pc.in ${CMAKE_BINARY_DIR}/${SUBPROJECT_NAME}.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/${SUBPROJECT_NAME}.pc DESTINATION lib/pkgconfig)

add_definitions(-DBASE_LOG_NAMESPACE=${SUBPROJECT_NAME})
install(FILES
    ${HEADERS}
    DESTINATION include/mars/plugins/${SUBPROJECT_NAME})

set(_INSTALL_DESTINATIONS
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION ${LIB_INSTALL_DIR}
	ARCHIVE DESTINATION lib
)


# Install the library into the lib folder
install(TARGETS ${SUBPROJECT_NAME} ${_INSTALL_DESTINATIONS})

# Install headers into mars include directory
install(FILES ${HEADERS} DESTINATION include/mars/plugins/${SUBPROJECT_NAME})

# Prepare and install necessary files to support finding of the library 
# using pkg-config
configure_file(${SUBPROJECT_NAME}.pc.in ${CMAKE_BINARY_DIR}/${SUBPROJECT_NAME}.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/${SUBPROJECT_NAME}.pc DESTINATION lib/pkgconfig)

add_definitions(-DBASE_LOG_NAMESPACE=${SUBPROJECT_NAME})
