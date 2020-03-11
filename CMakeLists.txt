##---------------------------------------------------------------------------------------------------------------------
##  FLOW
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2020 - Pablo Ramon Soria (a.k.a. Bardo91) 
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
##  and associated documentation files (the "Software"), to deal in the Software without restriction, 
##  including without limitation the rights to use, copy, modify, merge, publish, distribute, 
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial 
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES 
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
PROJECT(flow VERSION 0.1.0)


#########################################
######			General config	   ######
#########################################
option(BUILD_EXAMPLES   "Compile examples"              OFF)
option(BUILD_TESTS      "Prepare tests"                 OFF)
option(BUILD_TOOLS      "Build tools"                   OFF)

set(CMAKE_AUTOMOC ON)

##################################################
######    Library.                        ########
##################################################
file(GLOB_RECURSE FLOW_HEADERS "./include/*.h" "./include/*.inl")
file(GLOB_RECURSE FLOW_SOURCES "./src/*.cpp")

add_library(${PROJECT_NAME} SHARED ${FLOW_HEADERS} ${FLOW_SOURCES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE C)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC pthread)

##################################################
######    Loading 3rd party libraries.    ########
##################################################

find_package(NodeEditor)
if(${NodeEditor_FOUND})
  target_include_directories(${PROJECT_NAME} PUBLIC ${NodeEditor_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${NodeEditor_LIBRARIES})
  target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_QTNODEEDITOR)
endif()

find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
  target_include_directories(${PROJECT_NAME} PUBLIC "/usr/include/eigen3")
endif()

find_package(Boost REQUIRED COMPONENTS system filesystem)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC Boost::boost Boost::system Boost::filesystem)


##################################################
######    Documentation.                  ########
##################################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(	doc_flow
						${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
						WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
						COMMENT "Generating API documentation with Doxygen" VERBATIM )
endif(DOXYGEN_FOUND)

##################################################
######    Test.                           ########
##################################################
if(${BUILD_TESTS})
  find_package(GTest)
  if(${GTest_FOUND})
    enable_testing()
    add_subdirectory(tests)
  endif()
endif()

##################################################
######    Examples.                       ########
##################################################
if(${BUILD_EXAMPLES})
  add_subdirectory(examples)
endif()

##################################################
######    Tools.                          ########
##################################################
if(${BUILD_TOOLS})
  add_subdirectory(tools)
endif()

##################################################
######    Installation.                   ########
##################################################
include(CMakePackageConfigHelpers)

if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif() 

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets
	COMPONENT ${PROJECT_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

install(EXPORT ${PROJECT_NAME}-targets
	FILE "${PROJECT_NAME}-targets.cmake"
	NAMESPACE ${PROJECT_NAME}::
	DESTINATION lib/cmake/${PROJECT_NAME}
	COMPONENT ${PROJECT_NAME}
)

configure_file(
    "${CMAKE_SOURCE_DIR}/cmake/templates/${PROJECT_NAME}-config.cmake.in"
	"${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
	@ONLY
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    VERSION ${version}
    COMPATIBILITY AnyNewerVersion
)

install(
    FILES
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
    COMPONENT ${PROJECT_NAME}
)

# DESTINATION will be automatically prefixed by ${CMAKE_INSTALL_PREFIX}
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    COMPONENT ${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_PREFIX}
)

# uninstall target 
if(NOT TARGET uninstall) 
    configure_file( 
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/cmake_uninstall.cmake.in" 
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
        @ONLY) 
 
    add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
endif() 


