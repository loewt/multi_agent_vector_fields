@PACKAGE_INIT@


# Include the exported CMake file
get_filename_component(multi_agent_vector_fields_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

# This macro enables usage of find_dependency().
# https://cmake.org/cmake/help/v3.11/module/CMakeFindDependencyMacro.html
include(CMakeFindDependencyMacro)
find_package(Eigen3 REQUIRED)

# Declare the used packages in order to communicate the requirements upstream.
if(NOT TARGET multi_agent_vector_fields::multi_agent_vector_fields)
    include("${multi_agent_vector_fields_CMAKE_DIR}/multi_agent_vector_fields-config-targets.cmake")
    include("${multi_agent_vector_fields_CMAKE_DIR}/multi_agent_vector_fields-packages.cmake")
else()
    set(BUILD_TARGET multi_agent_vector_fields::multi_agent_vector_fields)

    get_target_property(TARGET_INCLUDE_DIRS ${BUILD_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
    set(TARGET_INCLUDE_DIRS "${TARGET_INCLUDE_DIRS}" CACHE PATH "${BUILD_TARGET} include directories")
    list(APPEND multi_agent_vector_fields_INCLUDE_DIRS ${TARGET_INCLUDE_DIRS})
endif()

check_required_components(multi_agent_vector_fields)