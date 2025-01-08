find_package(Eigen3 REQUIRED)
find_package(gafro REQUIRED)

add_library(${PROJECT_NAME} SHARED
    CfAgent.cpp
    GoalHeuristicCfAgent.cpp
    GoalObstacleHeuristicCfAgent.cpp
    HadHeuristicCfAgent.cpp
    ObstacleHeuristicCfAgent.cpp
    RandomCfAgent.cpp
    RealCfAgent.cpp
    VelocityHeuristicCfAgent.cpp
    CfManager.cpp
    Obstacle.cpp
)

# target_compile_options(${PROJECT_NAME}
# 	PUBLIC
#     "-Wall"
#     "-Wextra"
#     "-Werror=vla"
#     "-Wno-unused-function"
#     "-Wno-missing-braces"
#     "-Wno-unknown-pragmas"
#     "-Wno-parentheses"
#     "-pedantic"
#     "-Wconversion"
#     "-Werror=pedantic"
#     "-O2"
# )

include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${${PROJECT_NAME}_VERSION}
  SOVERSION 1
  INTERFACE_${PROJECT_NAME}_MAJOR_VERSION 1
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<INSTALL_INTERFACE:${${PROJECT_NAME}_PREFIX_INCLUDE}>
		# $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
		# $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}>
		# $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>/..
)

target_include_directories(${PROJECT_NAME} 
	SYSTEM PRIVATE
)

target_link_libraries(${PROJECT_NAME}
	PUBLIC
   Eigen3::Eigen
   gafro::gafro
)

install(
	FILES 
    CfAgent.hpp
    GoalHeuristicCfAgent.hpp
    GoalObstacleHeuristicCfAgent.hpp
    HadHeuristicCfAgent.hpp
    ObstacleHeuristicCfAgent.hpp
    RandomCfAgent.hpp
    RealCfAgent.hpp
    VelocityHeuristicCfAgent.hpp
    CfManager.hpp
    Obstacle.hpp
	DESTINATION 
		${${PROJECT_NAME}_PREFIX_INCLUDE}/${PROJECT_NAME} 
	COMPONENT 
		Devel
)