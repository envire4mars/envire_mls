set(SOURCES 
    EnvireMls.cpp
)

set(HEADERS
    EnvireMls.hpp
    defs.hpp
)

set(deps_pkg_config 
    lib_manager
    envire_core
    mars_sim
    envire_smurf_loader
    envire_fcl
    ${PCL_LIBS}
    base-types
    ode
    )

rock_library(envire_mls
    HEADERS ${HEADERS}
    SOURCES ${SOURCES}
    DEPS_PKGCONFIG ${deps_pkg_config}
    )

install(FILES
    ${HEADERS}
    DESTINATION include/mars/plugins/${PROJECT_NAME})