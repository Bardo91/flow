##---------------------------------------------------------------------------------------------------------------------
##  FLOW
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2019 - Pablo Ramon Soria (a.k.a. Bardo91) 
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
PROJECT(flow VERSION 0.0.1)


#########################################
######			General config	   ######
#########################################
option(BUILD_EXAMPLES "Compile examples" ON)
option(BUILD_TESTS "Prepare tests" OFF)

#########################################
######			Library	   ######
#########################################
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

#########################################
######			Dependencies       ######
#########################################

find_package(NodeEditor)
if(${NodeEditor_FOUND})
  target_include_directories(${PROJECT_NAME} PUBLIC ${NodeEditor_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${NodeEditor_LIBRARIES})
  target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_QTNODEEDITOR)
endif()

#########################################
######			Documentation	   ######
#########################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(	doc_flow
						${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
						WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
						COMMENT "Generating API documentation with Doxygen" VERBATIM )
endif(DOXYGEN_FOUND)

#########################################
######			Test				######
#########################################

if(${BUILD_TESTS})
  find_package(GTest)
  if(${GTest_FOUND})
    enable_testing()
    add_subdirectory(tests)
  endif()
endif()

#########################################
######			Test				######
#########################################

if(${BUILD_EXAMPLES})
  add_subdirectory(examples)
endif()

#########################################
######			INSTALL 		   ######
#########################################
include(CMakePackageConfigHelpers)


## INSTALLATION C++ Library
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif() 

install(TARGETS flow
        EXPORT flowTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}
        )

write_basic_package_version_file(
  flowConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
    )


# This installs the include folder
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.inl")

export(TARGETS flow NAMESPACE flow:: FILE flow.cmake)

# This generates flowTargets.cmake
install(EXPORT flowTargets
    FILE flowTargets.cmake
    NAMESPACE flow::
    DESTINATION lib/cmake/flow
    )

configure_file(flowConfig.cmake.in flowConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/flowConfig.cmake"
        DESTINATION lib/cmake/flow
        )

# uninstall target 
if(NOT TARGET uninstall) 
  configure_file( 
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" 
      "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
      IMMEDIATE @ONLY) 
 
  add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
endif() 


 
