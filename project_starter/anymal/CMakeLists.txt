# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/anymal_example)
ADD_EXECUTABLE (controller_anymal controller.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_anymal simviz.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller_anymal ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_anymal ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_anymal.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY anymal.urdf DESTINATION ${APP_RESOURCE_DIR})
