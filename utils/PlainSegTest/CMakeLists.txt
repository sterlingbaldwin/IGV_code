cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PlainSegTest)

find_package(PCL 1.2 REQUIRED)
SET(TRICLOPS "-ltriclops")
SET(PNMUTILS "-lpnmutils")
SET(FLYCAP "-lflycapture")
SET(FLYCAPBRIDGE "-lflycapture2bridge")
SET(PTHREADS "-lpthread")
SET(DL "-ldl")
SET(M "-lm")

include_directories(${PCL_INCLUDE_DIRS})
include_directories(/usr/include/triclops)
include_directories(/usr/include/flycapture)
include_directories(/usr/include/flycapture/C)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(${TRICLOPS})
add_definitions(${PNMUTILS})
add_definitions(${FLYCAP})
add_definitions(${FLYCAPBRIDGE})
add_definitions(${PTHREADS})
add_definitions(${DL})
add_definitions(${M})



add_executable (PlainSegTest 
				PlainSegTest.cpp 
				)

target_link_libraries (PlainSegTest ${PCL_LIBRARIES} 
						/usr/lib/libtriclops.so /usr/lib/libpnmutils.a 
						/usr/lib/libflycapture.so 
						/usr/lib/libflycapture2bridge.so 
						/usr/lib/i386-linux-gnu/libpthread.so
)