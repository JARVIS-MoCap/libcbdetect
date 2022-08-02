project(cbdetect)

add_library(${PROJECT_NAME}
  src/board_energy.cpp
  src/board_energy.h
  src/boards_from_corners.cpp
  src/boards_from_corners.h
  src/config.h
  src/create_correlation_patch.cpp
  src/create_correlation_patch.h
  src/filter_board.cpp
  src/filter_board.h
  src/filter_corners.cpp
  src/filter_corners.h
  src/find_corners.cpp
  src/find_corners.h
  src/find_modes_meanshift.cpp
  src/find_modes_meanshift.h
  src/get_image_patch.cpp
  src/get_image_patch.h
  src/get_init_location.cpp
  src/get_init_location.h
  src/grow_board.cpp
  src/grow_board.h
  src/image_normalization_and_gradients.cpp
  src/image_normalization_and_gradients.h
  src/init_board.cpp
  src/init_board.h
  src/non_maximum_suppression.cpp
  src/non_maximum_suppression.h
  src/polynomial_fit.cpp
  src/polynomial_fit.h
  src/refine_corners.cpp
  src/refine_corners.h
  src/score_corners.cpp
  src/score_corners.h
  src/weight_mask.cpp
  src/weight_mask.h
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${PROJECT_SOURCE_DIR}
    ../
)

target_link_libraries(${PROJECT_NAME}
	opencv_core
	opencv_calib3d
	opencv_imgproc
)

#target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic -Wunused)
