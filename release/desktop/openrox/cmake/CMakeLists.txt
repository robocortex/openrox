project(OPENROX_EXAMPLES C CXX)

# Cmake minimal version required to use this script
cmake_minimum_required(VERSION 2.8)

# Set openrox link/includes
include_directories(${OPENROX_EXAMPLES_SOURCE_DIR}/inc)
find_library(OPENROX_LIBRARY openrox PATH ${OPENROX_EXAMPLES_SOURCE_DIR}/bin/ ${OPENROX_EXAMPLES_SOURCE_DIR}/lib/ NO_DEFAULT_PATH)

# Parameters
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OPENROX_EXAMPLES_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OPENROX_EXAMPLES_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OPENROX_EXAMPLES_SOURCE_DIR}/bin)
foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${OPENROX_EXAMPLES_SOURCE_DIR}/bin)
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${OPENROX_EXAMPLES_SOURCE_DIR}/bin)
    set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${OPENROX_EXAMPLES_SOURCE_DIR}/bin)
endforeach( OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES )

# List of examples

add_executable(rox_example_camera_calibration ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_camera_calibration.c)
target_link_libraries(rox_example_camera_calibration ${OPENROX_LIBRARY})

add_executable(rox_example_database_generation ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_database_generation.c)
target_link_libraries(rox_example_database_generation ${OPENROX_LIBRARY})

add_executable(rox_example_image_undistortion ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_image_undistortion.c)
target_link_libraries(rox_example_image_undistortion ${OPENROX_LIBRARY})

add_executable(rox_example_odometry_single_plane_database ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_odometry_single_plane_database.c)
target_link_libraries(rox_example_odometry_single_plane_database ${OPENROX_LIBRARY})

add_executable(rox_example_odometry_database_compiled ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_odometry_database_compiled.c)
target_link_libraries(rox_example_odometry_database_compiled ${OPENROX_LIBRARY})

add_executable(rox_example_odometry_database_features ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_odometry_database_features.c)
target_link_libraries(rox_example_odometry_database_features ${OPENROX_LIBRARY})

#add_executable(rox_example_identification ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_identification.c)
#target_link_libraries(rox_example_identification ${OPENROX_LIBRARY})

add_executable(rox_example_identification_database_cloud ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_identification_database_cloud.c)
target_link_libraries(rox_example_identification_database_cloud ${OPENROX_LIBRARY})

add_executable(rox_example_odometry_inertial_observer ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_odometry_inertial_observer.c)
target_link_libraries(rox_example_odometry_inertial_observer ${OPENROX_LIBRARY})

add_executable(rox_example_odometry_single_plane_photoframe ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_odometry_single_plane_photoframe.c)
target_link_libraries(rox_example_odometry_single_plane_photoframe ${OPENROX_LIBRARY})

add_executable(rox_example_odometry_single_plane ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_odometry_single_plane.c)
target_link_libraries(rox_example_odometry_single_plane ${OPENROX_LIBRARY})

add_executable(rox_example_odometry_multi_plane ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_odometry_multi_plane.c)
target_link_libraries(rox_example_odometry_multi_plane ${OPENROX_LIBRARY})

add_executable(rox_example_image_score ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_image_score.c)
target_link_libraries(rox_example_image_score ${OPENROX_LIBRARY})

add_executable(rox_example_tracking ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_tracking.c)
target_link_libraries(rox_example_tracking ${OPENROX_LIBRARY})

add_executable(rox_example_tracking_photoframe ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_tracking_photoframe.c)
target_link_libraries(rox_example_tracking_photoframe ${OPENROX_LIBRARY})

add_executable(rox_example_motion_detection ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_motion_detection.c)
target_link_libraries(rox_example_motion_detection ${OPENROX_LIBRARY})

add_executable(rox_example_rectangle_detection ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_rectangle_detection.c)
target_link_libraries(rox_example_rectangle_detection ${OPENROX_LIBRARY})

add_executable(rox_example_plane_detection_from_rectangles ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_plane_detection_from_rectangles.c)
target_link_libraries(rox_example_plane_detection_from_rectangles ${OPENROX_LIBRARY})

add_executable(rox_example_odometry_single_plane_filter_matse3 ${OPENROX_EXAMPLES_SOURCE_DIR}/src/rox_example_odometry_single_plane_filter_matse3.c)
target_link_libraries(rox_example_odometry_single_plane_filter_matse3 ${OPENROX_LIBRARY})
