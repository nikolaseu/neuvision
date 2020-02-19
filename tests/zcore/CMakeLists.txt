project(ZCoreTests LANGUAGES CXX)

find_package(Qt5 COMPONENTS Test REQUIRED)

macro(add_zcore_test testname)
    add_executable(${PROJECT_NAME}_${testname}
      ${testname}
    )
    target_link_libraries(${PROJECT_NAME}_${testname} PRIVATE
        Qt5::Test
        ZCore)
    add_test(NAME ${PROJECT_NAME}_${testname} COMMAND $<TARGET_FILE:${PROJECT_NAME}_${testname}>)
endmacro()

add_zcore_test(tst_zsettingsitem.cpp)
