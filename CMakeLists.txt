#Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.

project(openimu_plugin C CXX)

include_directories(${SAMPLE_PLUGINS_COMMON_DIR})
include_directories(include)

#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------
set(PUBLIC_DOCS
    README.md
)

set(SOURCES
    src/main.cpp
    include/openimu300.h
    ${SAMPLE_PLUGINS_COMMON_DIR}/ByteQueue.hpp
    ${SAMPLE_PLUGINS_COMMON_DIR}/BufferPool.hpp
    )

set(LIBRARIES
    ${Driveworks_LIBRARIES}
)

#-------------------------------------------------------------------------------
# Final target
#-------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Samples")

# ------------------------------------------------------------------------------
# Install target
# ------------------------------------------------------------------------------
get_property(TARGET_PACKAGE GLOBAL PROPERTY TARGET_PACKAGE)
install(TARGETS ${PROJECT_NAME}
    COMPONENT dw-samples-${TARGET_PACKAGE} # targeted package (dw / dav / ..)
    DESTINATION ${SDK_SAMPLE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    COMPONENT dw-tests
    DESTINATION ${SDK_SAMPLE_DESTINATION}
)
