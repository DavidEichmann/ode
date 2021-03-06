project(Unit_Tests)

set( HDRS
		../library/include/ProceduralStableHeaders.h
		../samples/common/include/BaseApplication.h
		include/Tests.h
	)

set( SRCS
		../library/src/ProceduralPrecompiledHeaders.cpp
		../samples/common/src/BaseApplication.cpp
		src/Tests.cpp
	)

include_directories( ${OIS_INCLUDE_DIRS}
	${OGRE_INCLUDE_DIRS}
	../samples/common/include
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(Unit_Tests WIN32 ${HDRS} ${SRCS})

set_target_properties(Unit_Tests PROPERTIES DEBUG_POSTFIX _d)

target_link_libraries(Unit_Tests ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${OgreProcedural_LIBRARIES})

procedural_create_vcproj_userfile(Unit_Tests)

# specify a precompiled header to use
use_precompiled_header(Unit_Tests 
  "../library/include/ProceduralStableHeaders.h"
  "../library/src/ProceduralPrecompiledHeaders.cpp"
)