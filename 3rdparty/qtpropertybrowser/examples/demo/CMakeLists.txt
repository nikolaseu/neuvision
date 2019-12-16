
SET(example_name demo)

SET(KIT_SRCS
  main.cpp
  )
  
SET(KIT_resources
  demo.qrc
  )
  
INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  )
  
QT4_ADD_RESOURCES(KIT_QRC_SRCS ${KIT_resources})
  
ADD_EXECUTABLE(${example_name} ${KIT_SRCS} ${KIT_QRC_SRCS})
TARGET_LINK_LIBRARIES(${example_name} ${PROJECT_NAME})
