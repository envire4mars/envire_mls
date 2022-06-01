set(SOURCES 
    src/EnvireMls.cpp
    PARENT_SCOPE
)

set(HEADERS
    src/EnvireMls.hpp
    src/defs.hpp
    PARENT_SCOPE
)

rock_library  (envire_mls
        HEADERS ${HEADERS}
        SOURCES ${SOURCES}
        DEPS_PKGCONFIG ${deps_pkg_config})

install(FILES
    ${HEADERS}
    DESTINATION include/mars/plugins/${PROJECT_NAME})