macro(cl_to_const_char cl_filename cl_const_char_name)
    message(STATUS "${cl_filename} -> ${cl_const_char_name}")

    FILE(STRINGS ${cl_filename} cl_content)

    FILE(REMOVE "${cl_const_char_name}")
    foreach(cl_string ${cl_content})
        STRING(REPLACE "\\" "\\\\" out_cl1 "${cl_string}")
        string(REPLACE "\"" "\\\"" out_cl2 "${out_cl1}")
        FILE(APPEND "${cl_const_char_name}" "\"${out_cl2}\\n\"\n")
    endforeach()
endmacro()

if (CMAKE_ARGV2)
    cl_to_const_char(${CMAKE_ARGV1} ${CMAKE_ARGV2})
endif()

