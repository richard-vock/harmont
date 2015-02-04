###############################################################################
# Find RGBE
#
# This sets the following variables:
# RGBE_FOUND - True if RGBE was found.
# RGBE_INCLUDE_DIRS - Directories containing the RGBE include files.
# RGBE_LIBRARY_DIRS - Directories containing the RGBE library.
# RGBE_LIBRARIES - RGBE library files.

find_path(RGBE_INCLUDE_DIR rgbe
    HINTS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}")

find_library(RGBE_LIBRARY_PATH rgbe HINTS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib")

if(EXISTS ${RGBE_LIBRARY_PATH})
get_filename_component(RGBE_LIBRARY ${RGBE_LIBRARY_PATH} NAME)
find_path(RGBE_LIBRARY_DIR ${RGBE_LIBRARY} HINTS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib")
endif()

set(RGBE_INCLUDE_DIRS ${RGBE_INCLUDE_DIR})
set(RGBE_LIBRARY_DIRS ${RGBE_LIBRARY_DIR})
set(RGBE_LIBRARIES ${RGBE_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RGBE DEFAULT_MSG RGBE_INCLUDE_DIR RGBE_LIBRARY RGBE_LIBRARY_DIR)

mark_as_advanced(RGBE_INCLUDE_DIR)
mark_as_advanced(RGBE_LIBRARY_DIR)
mark_as_advanced(RGBE_LIBRARY)
mark_as_advanced(RGBE_LIBRARY_PATH)
