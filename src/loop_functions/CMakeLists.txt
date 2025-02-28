# Set sources for the loop functions
set(LOOP_FUNCTIONS_SOURCES
    ../Arena.cpp
    SensorDegLoopFunctions.cpp
)

if(BUILD_REAL_KHEPERAIV_EXPERIMENT_LOOP_FUNCTIONS)
    set(LOOP_FUNCTIONS_SOURCES
        ${LOOP_FUNCTIONS_SOURCES}
        RealKheperaIVExperimentLoopFunctions.cpp
    )
endif()

# Add loop functions library
add_library(${PROJECT_NAME}_loop_functions
    SHARED
    ${LOOP_FUNCTIONS_SOURCES}
)

if(BUILD_REAL_KHEPERAIV_EXPERIMENT_LOOP_FUNCTIONS)
    target_link_libraries(${PROJECT_NAME}_loop_functions
        PRIVATE
        ${PROJECT_NAME}_algorithms
        ${PROJECT_NAME}_controllers
        ${PROJECT_NAME}_messages
        vicon_sdk
    )

    target_include_directories(${PROJECT_NAME}_loop_functions
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/vicon_sdk>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/real_kheperaiv_controller/sensor-degradation-filter-real-kheperaiv/src>
    )
else()
    target_link_libraries(${PROJECT_NAME}_loop_functions
        PRIVATE
        ${PROJECT_NAME}_algorithms
        ${PROJECT_NAME}_controllers
    )

    target_include_directories(${PROJECT_NAME}_loop_functions
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/vicon_sdk>
    )
endif()