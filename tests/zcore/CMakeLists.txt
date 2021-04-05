project(ZCoreTests LANGUAGES CXX)

find_package(QT NAMES Qt5 Qt6 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Test REQUIRED)

macro(add_zcore_test testname)
    add_executable(${PROJECT_NAME}_${testname}
        ${testname}
    )
    target_link_libraries(${PROJECT_NAME}_${testname} PRIVATE
        Qt::Test
        ZCore
    )
    add_test(NAME ${PROJECT_NAME}_${testname} COMMAND $<TARGET_FILE:${PROJECT_NAME}_${testname}>)
endmacro()

add_zcore_test(tst_zsettingsitem.cpp)
