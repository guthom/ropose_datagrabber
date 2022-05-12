set(COMPONENT_NAME "dataelements")

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
set(LOCAL_LINKLIBS)

#Create Libname list
list(APPEND LIBNAMES)

#add additional node
#SET(LOCNODENAME Datagrabber)
#add_executable(${LOCNODENAME} ${LOCNODENAME}.cpp)
#add_dependencies(${LOCNODENAME} ${GLOB_DEPENDENCIES})
#target_link_libraries(${LOCNODENAME} ${LINKLIBS} ${GLOB_LINKLIBS})

set(GLOB_DEPENDENCIES
    ${GLOB_DEPENDENCIES} 
    ${LIBNAME} 
    PARENT_SCOPE
)

set(GLOB_LINKLIBS
   ${GLOB_LINKLIBS}
   ${LIBNAMES}
   PARENT_SCOPE
)