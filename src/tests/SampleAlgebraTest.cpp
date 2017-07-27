//
// Created by frivas on 27/07/17.
//

#include "SampleAlgebraTest.h"


CPPUNIT_TEST_SUITE_REGISTRATION( SampleAlgebraTest );

void SampleAlgebraTest::setUp() {

}

void SampleAlgebraTest::tearDown() {

}


void SampleAlgebraTest::SummCommutativeTest() {

    CPPUNIT_ASSERT_EQUAL(5+3, 3+5);
}


void SampleAlgebraTest::DivCommutativeTest() {

    CPPUNIT_ASSERT_EQUAL(5/3, 3/5);
}
