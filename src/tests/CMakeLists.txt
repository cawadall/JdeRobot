include_directories(${CPPUNIT_INCLUDE_DIR})



add_executable(jderbot_tests_test main.cpp SampleAlgebraTest)

target_link_libraries(jderbot_tests_test
        ${CPPUNIT_LIBRARY}
        )