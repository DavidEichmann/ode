# Install script for directory: /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee]|[Nn][Oo][Nn][Ee]|)$")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so")
      FILE(RPATH_CHECK
           FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so"
           RPATH "")
    ENDIF()
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/lib/libOgreProcedural.so")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF()
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee]|[Nn][Oo][Nn][Ee]|)$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so")
      FILE(RPATH_CHECK
           FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so"
           RPATH "")
    ENDIF()
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/lib/libOgreProcedural.so")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF()
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so")
      FILE(RPATH_CHECK
           FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so"
           RPATH "")
    ENDIF()
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/lib/libOgreProcedural.so")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF()
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so")
      FILE(RPATH_CHECK
           FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so"
           RPATH "")
    ENDIF()
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/lib/libOgreProcedural.so")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOgreProcedural.so")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF()
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OgreProcedural" TYPE FILE FILES
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralPlatform.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/Procedural.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralMeshGenerator.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralBoxGenerator.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralCapsuleGenerator.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralConeGenerator.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralCylinderGenerator.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralIcoSphereGenerator.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralPlaneGenerator.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralSphereGenerator.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralRoot.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralRoundedBoxGenerator.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralTorusGenerator.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralTorusKnotGenerator.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralTubeGenerator.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralUtils.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralExtruder.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralLathe.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralShape.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralShapeGenerators.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralPath.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralPathGenerators.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralTrack.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralSplines.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralTriangulator.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralTriangleBuffer.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralStableHeaders.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralMultiShape.h"
    "/home/david/git/ODE_01/ODE_01/lib/ogre-procedural/library/include/ProceduralGeometryHelpers.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

