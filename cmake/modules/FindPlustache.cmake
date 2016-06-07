###############################################################################
# Find Plustache
#
# This sets the following variables:
# PLUSTACHE_FOUND - True if Plustache was found.
# PLUSTACHE_INCLUDE_DIRS - Directories containing the Plustache include files.
# PLUSTACHE_LIBRARY_DIRS - Directories containing the Plustache library.
# PLUSTACHE_LIBRARIES - Plustache library files.

find_path(PLUSTACHE_INCLUDE_DIR plustache
    PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATH)

find_library(PLUSTACHE_LIBRARY_PATH plustache PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATH)

if(EXISTS ${PLUSTACHE_LIBRARY_PATH})
get_filename_component(PLUSTACHE_LIBRARY ${PLUSTACHE_LIBRARY_PATH} NAME)
find_path(PLUSTACHE_LIBRARY_DIR ${PLUSTACHE_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATH)
endif()

set(PLUSTACHE_INCLUDE_DIRS ${PLUSTACHE_INCLUDE_DIR})
set(PLUSTACHE_LIBRARY_DIRS ${PLUSTACHE_LIBRARY_DIR})
set(PLUSTACHE_LIBRARIES ${PLUSTACHE_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Plustache DEFAULT_MSG PLUSTACHE_INCLUDE_DIR PLUSTACHE_LIBRARY PLUSTACHE_LIBRARY_DIR)

mark_as_advanced(PLUSTACHE_INCLUDE_DIR)
mark_as_advanced(PLUSTACHE_LIBRARY_DIR)
mark_as_advanced(PLUSTACHE_LIBRARY)
mark_as_advanced(PLUSTACHE_LIBRARY_PATH)
