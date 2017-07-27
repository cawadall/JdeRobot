//
// Created by frivas on 27/07/17.
//

#ifndef JDEROBOT_SAMPLEALGEBRATEST_H
#define JDEROBOT_SAMPLEALGEBRATEST_H

#include <cppunit/extensions/HelperMacros.h>


class SampleAlgebraTest:public CPPUNIT_NS::TestFixture {
    CPPUNIT_TEST_SUITE(SampleAlgebraTest);

    CPPUNIT_TEST(SummCommutativeTest);
    CPPUNIT_TEST(DivCommutativeTest);


    CPPUNIT_TEST_SUITE_END();



public:
    void setUp();
    void tearDown();

    void SummCommutativeTest();
    void DivCommutativeTest();



};


#endif //JDEROBOT_SAMPLEALGEBRATEST_H
