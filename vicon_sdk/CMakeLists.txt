add_library(vicon_sdk SHARED IMPORTED GLOBAL)
set_property(TARGET vicon_sdk PROPERTY
    IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/vicon_sdk/libViconDataStreamSDK_CPP.so")