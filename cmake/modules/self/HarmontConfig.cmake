###############################################################################
# Find Harmont
#
# This sets the following variables:
# HARMONT_FOUND - True if HARMONT was found.
# HARMONT_INCLUDE_DIRS - Directories containing the HARMONT include files.
# HARMONT_LIBRARY_DIRS - Directories containing the HARMONT library.
# HARMONT_LIBRARIES - HARMONT library files.

if(WIN32)
    find_path(HARMONT_INCLUDE_DIR harmont PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)

    find_library(HARMONT_LIBRARY_PATH harmont PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${HARMONT_LIBRARY_PATH})
        get_filename_component(HARMONT_LIBRARY ${HARMONT_LIBRARY_PATH} NAME)
        find_path(HARMONT_LIBRARY_DIR ${HARMONT_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)
    endif()
else(WIN32)
    find_path(HARMONT_INCLUDE_DIR harmont PATHS "/usr/include" "/usr/local/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)
    find_library(HARMONT_LIBRARY_PATH harmont PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${HARMONT_LIBRARY_PATH})
        get_filename_component(HARMONT_LIBRARY ${HARMONT_LIBRARY_PATH} NAME)
        find_path(HARMONT_LIBRARY_DIR ${HARMONT_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)
    endif()
endif(WIN32)

set(HARMONT_INCLUDE_DIRS ${HARMONT_INCLUDE_DIR})
set(HARMONT_LIBRARY_DIRS ${HARMONT_LIBRARY_DIR})
set(HARMONT_LIBRARIES ${HARMONT_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(HARMONT DEFAULT_MSG HARMONT_INCLUDE_DIR HARMONT_LIBRARY HARMONT_LIBRARY_DIR)

mark_as_advanced(HARMONT_INCLUDE_DIR)
mark_as_advanced(HARMONT_LIBRARY_DIR)
mark_as_advanced(HARMONT_LIBRARY)
mark_as_advanced(HARMONT_LIBRARY_PATH)
