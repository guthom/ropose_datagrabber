set(COMPONENT_NAME "datagrabber")

set (SUBMODULES 
    ${COMPONENT_NAME} 
    PARENT_SCOPE
)

message("Add sub module: ${COMPONENT_NAME}")

file(GLOB SUB_SOURCE *.cpp )
file(GLOB SUB_HEADERS *.h)

message("Add sub sources of Module: ${COMPONENT_NAME}:")
message("${SUB_SOURCE}")

message("Add sub headers of Module: ${COMPONENT_NAME}:")
message("${SUB_HEADERS}")

set(SOURCE
   ${SOURCE}
   ${SUB_SOURCE}
   PARENT_SCOPE
)

set(HEADERS
   ${HEADERS}
   ${SUB_HEADERS}
   PARENT_SCOPE
)
#Add Local Dependencies
set(LOCAL_DEPENDENCIES)
set(LOCAL_LINKLIBS ropose_datagrabber_helper)

#Create Libname list
list(APPEND LIBNAMES
        dataelements/FrameBase.cpp
        dataelements/FrameRGB.cpp
        dataelements/FrameDepth.cpp
        dataelements/FramePC.cpp
        dataelements/FrameIR.cpp
        dataelements/Camera.cpp
        dataelements/Grabcollection.cpp
        dataelements/Grabelement.cpp
        dataelements/Recorder.cpp
        dataelements/TFLogger.cpp
        dataelements/PointProjector.cpp
        dataelements/GazeboLinkStates.cpp
        )

#add additional node
SET(LOCNODENAME datagrabber)
add_executable(${LOCNODENAME} ${LOCNODENAME}.cpp ${LIBNAMES})
add_dependencies(${LOCNODENAME} ${GLOB_DEPENDENCIES})
target_link_libraries(${LOCNODENAME} ${LOCAL_LINKLIBS} ${GLOB_LINKLIBS})

set(GLOB_DEPENDENCIES
    ${GLOB_DEPENDENCIES} 
    ${LOCAL_DEPENDENCIES}
    PARENT_SCOPE
)

set(GLOB_LINKLIBS
   ${GLOB_LINKLIBS}
   ${LOCAL_LINKLIBS}
   PARENT_SCOPE
)

add_subdirectory(dataelements/)


