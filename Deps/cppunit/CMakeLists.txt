FIND_PATH(CPPUNIT_INCLUDE_DIR cppunit/Test.h)
FIND_LIBRARY(CPPUNIT_LIBRARY NAMES cppunit)
set(CppUnit_FIND_REQUIRED true)

IF (CPPUNIT_INCLUDE_DIR AND CPPUNIT_LIBRARY)
    SET(CPPUNIT_FOUND TRUE)
ENDIF (CPPUNIT_INCLUDE_DIR AND CPPUNIT_LIBRARY)

IF (CPPUNIT_FOUND)

    # show which CppUnit was found only if not quiet
    IF (NOT CppUnit_FIND_QUIETLY)
        MESSAGE(STATUS "Found CppUnit: ${CPPUNIT_LIBRARY}")
    ENDIF (NOT CppUnit_FIND_QUIETLY)

ELSE (CPPUNIT_FOUND)

    # fatal error if CppUnit is required but not found
    IF (CppUnit_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "Could not find CppUnit (libcppunit-dev required)")
    ENDIF (CppUnit_FIND_REQUIRED)

ENDIF (CPPUNIT_FOUND)


list(APPEND DEPS_DEV libcppunit-dev)
