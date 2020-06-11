
# call pkg_check_modules default in cross_compile is wrong _INCLUDE_DIRS, it always point to host folder
# we need to wrap by add sysroot path to prefix

find_package(PkgConfig)

macro(pkg_check_modules_wrapper _prefix module0)
    pkg_check_modules(${_prefix} REQUIRED ${module0})
    if(${_prefix}_FOUND AND DEFINED CMAKE_SYSROOT)
        foreach(path ${${_prefix}_INCLUDE_DIRS})
            if(NOT ${path} MATCHES ".*sysroot")
                set(${_prefix}_INCLUDE_DIRS_tmp ${${_prefix}_INCLUDE_DIRS_tmp} ${CMAKE_SYSROOT}/${path})
            endif()
        endforeach()
        if(DEFINED ${_prefix}_INCLUDE_DIRS_tmp)
            set(${_prefix}_INCLUDE_DIRS ${${_prefix}_INCLUDE_DIRS_tmp})
        endif()
    endif()
endmacro()