//
// Created by frivas on 27/07/17.
//

#include <cppunit/BriefTestProgressListener.h>
#include <cppunit/CompilerOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/TestResult.h>
#include <cppunit/TestResultCollector.h>
#include <cppunit/TestRunner.h>
#include <cppunit/XmlOutputter.h>
#include "SampleAlgebraTest.h"
#include <cppunit/ui/text/TestRunner.h>

int main(int argc, char* argv[])
{

    // Create the event manager and test controller
    CPPUNIT_NS::TestResult controller;

    // Add a listener that collects test result
    CPPUNIT_NS::TestResultCollector result;
    controller.addListener(&result);

    // Add a listener that prints dots as tests run.
    CPPUNIT_NS::BriefTestProgressListener progress;
    controller.addListener(&progress);

    // Add the top suite to the test runner
    CPPUNIT_NS::TestRunner runner;
    runner.addTest(CPPUNIT_NS::TestFactoryRegistry::getRegistry().makeTest());
    runner.run(controller);

#ifdef USEXUNIT
    // Print test results in xml format (Used by jenkins xunit plugin).
        std::cout << "USEXUNIt is on" << std::endl;
        std::ofstream xmlout ( "testRoofModelLib.xml" );
        CPPUNIT_NS::XmlOutputter xmlOutputter ( &result, xmlout );
        xmlOutputter.write();
#else
    std::cout << "USEXUNIt is off" << std::endl;
    // Print test in a compiler compatible format.
    CPPUNIT_NS::CompilerOutputter outputter(&result, CPPUNIT_NS::stdCOut());
    outputter.write();
#endif // USEXUNIT


    //getchar();
    return 0;

}