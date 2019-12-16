

SET(KIT_SRCS
  qtbuttonpropertybrowser.cpp
  qteditorfactory.cpp
  qtgroupboxpropertybrowser.cpp
  qtpropertybrowser.cpp
  qtpropertybrowserutils.cpp
  qtpropertymanager.cpp
  qttreepropertybrowser.cpp
  qtvariantproperty.cpp
  )
  
SET(KIT_H
	qtbuttonpropertybrowser.h
	qteditorfactory.h
	qtgroupboxpropertybrowser.h
	qtpropertybrowser.h
	qtpropertybrowserutils_p.h #===> also in KIT_MOC_SRCS, think about it...
	qtpropertymanager.h
	qttreepropertybrowser.h
	qtvariantproperty.h
)

SET(KIT_MOC_SRCS
  qtpropertybrowserutils_p.h
  )

SET(KIT_UI_FORMS
  )

SET(KIT_resources
  qtpropertybrowser.qrc
  )
  
INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  )
  
qt5_wrap_ui(KIT_UI_CPP ${KIT_UI_FORMS})
qt5_wrap_cpp(KIT_MOC_CPP ${KIT_MOC_SRCS})
qt5_add_resources(KIT_QRC_SRCS ${KIT_resources})

SET(libname ${PROJECT_NAME})
add_library(${libname} STATIC
  ${KIT_H}
  ${KIT_SRCS}
  ${KIT_UI_CPP}
  ${KIT_MOC_CPP}
  ${KIT_QRC_SRCS}
  )
  
install(TARGETS ${libname} DESTINATION lib)
install(FILES ${KIT_H} DESTINATION include/${PROJECT_NAME})
  
target_link_libraries(
  ${libname}
  PRIVATE
  Qt5::Widgets
  )

target_compile_definitions(${libname} PRIVATE QT_DISABLE_DEPRECATED_BEFORE=0)

# List of header that should go though moc
SET(KIT_HEADERS_MOC_SRCS
  qtbuttonpropertybrowser.h
  qteditorfactory.h
  qtgroupboxpropertybrowser.h
  qtpropertybrowser.h
  qtpropertybrowserutils_p.h
  qtpropertymanager.h
  qttreepropertybrowser.h
  qtvariantproperty.h
  )

SET(KIT_MOC_CPP)
FOREACH(file ${KIT_HEADERS_MOC_SRCS})
  get_filename_component(filename_we ${file} NAME_WE)
  SET(output_file moc_${filename_we}.cpp)
  QT5_GENERATE_MOC(${file} ${output_file})
  LIST(APPEND KIT_MOC_CPP ${output_file})
ENDFOREACH()

# List of cpp files that should go though moc
SET(KIT_CPP_MOC_SRCS
  qteditorfactory.cpp
  qtpropertymanager.cpp
  qttreepropertybrowser.cpp
  )

FOREACH(file ${KIT_CPP_MOC_SRCS})
  get_filename_component(filename_we ${file} NAME_WE)
  SET(output_file ${filename_we}.moc)
  QT5_GENERATE_MOC(${file} ${output_file})
  LIST(APPEND KIT_MOC_CPP ${output_file})
ENDFOREACH()

add_custom_target(${libname}GenerateMoc
  DEPENDS ${KIT_MOC_CPP}
  )
add_dependencies(${libname} ${libname}GenerateMoc)
