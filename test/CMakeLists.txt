set(SUBPROJECT_NAME envire_mls_tests)

include(FindPkgConfig)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PKGCONFIG REQUIRED
			    lib_manager
				yaml-cpp
			    mars_sim
                ${PCL_LIBS}
				envire_core
)

set(deps_pkg_config 
    lib_manager
    yaml-cpp
    mars_sim
    ${PCL_LIBS}
    envire_core
    )

set(SOURCES 
	EnvireMlsTests.cpp
)
set(HEADERS
	EnvireMlsTests.hpp
	defs.hpp
	constants.hpp
)


rock_library(${SUBPROJECT_NAME}
        HEADERS ${HEADERS}
        SOURCES ${SOURCES}
        DEPS_PKGCONFIG ${deps_pkg_config}
        DEPS envire_mls)

# Install headers into mars include directory
install(FILES ${HEADERS} DESTINATION include/mars/plugins/${SUBPROJECT_NAME})
