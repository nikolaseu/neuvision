
SET(example_name decoration)

SET(KIT_SRCS
  main.cpp
  )
  
INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

QT4_WRAP_CPP(KIT_MOC_CPP ${KIT_MOC_SRCS})
  
ADD_EXECUTABLE(${example_name} ${KIT_SRCS} ${KIT_MOC_SRCS})
TARGET_LINK_LIBRARIES(${example_name} ${PROJECT_NAME})

SET(KIT_CPP_MOC_SRCS
  main.cpp
  )

SET(KIT_MOC_CPP)
FOREACH(file ${KIT_CPP_MOC_SRCS})
  get_filename_component(filename_we ${file} NAME_WE)
  SET(output_file ${filename_we}.moc)
  QT4_GENERATE_MOC(${file} ${output_file})
  LIST(APPEND KIT_MOC_CPP ${output_file})
ENDFOREACH()

add_custom_target(${example_name}GenerateMoc
  DEPENDS ${KIT_MOC_CPP}
  )
add_dependencies(${example_name} ${example_name}GenerateMoc)
