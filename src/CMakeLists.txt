SET( TARGET_NAME LibraryTemplate )
SET( SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR} )

SET( SOURCES
    ${SOURCE_DIR}/dummy.cpp
)

SET( OPENCL_SOURCES
    ${SOURCE_DIR}/opencl/dummy.cl
)
SOURCE_GROUP( OpenCL FILES ${OPENCL_SOURCES} )

ADD_LIBRARY( ${TARGET_NAME} ${SOURCES} ${OPENCL_SOURCES} )

foreach(cl ${OpenCL_sources})
    # 1. add command to generate const char * string from OpenCL file
    get_filename_component(cl_basename ${cl} NAME)
    set(cl_const_char_name "${CMAKE_CURRENT_BINARY_DIR}/${cl_basename}.c")
    
    add_custom_command(OUTPUT ${cl_const_char_name}
                       COMMAND ${CMAKE_COMMAND} ${cl} ${cl_const_char_name} -P ${CMAKE_MODULE_PATH}/OCL2C.cmake
                       MAIN_DEPENDENCY ${cl}
    )
    
    # 2. add to the library the dependency on that const char *file
    add_dependencies(${TARGET_NAME} ${cl_const_char_name})
endforeach()

TARGET_LINK_LIBRARIES( ${TARGET_NAME} ${OPENCL_LIBRARIES} ${OpenCV_LIBRARIES} )

