
SET(example_name canvas_variant)

SET(KIT_SRCS
  main.cpp
  mainwindow.cpp
  mainwindow.h
  qtcanvas.cpp
  qtcanvas.h
  )

SET(KIT_MOC_SRCS
  mainwindow.h
  qtcanvas.h
  )
  
INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

QT4_WRAP_CPP(KIT_MOC_CPP ${KIT_MOC_SRCS})
  
ADD_EXECUTABLE(${example_name} ${KIT_SRCS} ${KIT_MOC_CPP})
TARGET_LINK_LIBRARIES(${example_name} ${PROJECT_NAME})
