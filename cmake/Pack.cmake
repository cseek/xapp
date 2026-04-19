file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/pack)
set(CPACK_GENERATOR "DEB;ZIP;TGZ")
set(CPACK_PACKAGE_NAME "${PACKAGE_ID}")
set(CPACK_PACKAGE_VENDOR "${PACKAGE_VENDOR}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PACKAGE_SUMMARY}")
set(CPACK_PACKAGE_VERSION "${CMAKE_PROJECT_VERSION}")
set(CPACK_PACKAGING_INSTALL_PREFIX "/")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l|armv7")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i386|i686")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "i386")
else()
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "${PACKAGE_MAINTAINER}")
#set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
# Include maintainer scripts (postinst/prerm) in the DEBIAN control directory
# set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
#    "${CMAKE_BINARY_DIR}/pack/postinst;${CMAKE_BINARY_DIR}/pack/prerm"
#)
# When packaging non-debian archives (e.g. ZIP), stage files under a DESTDIR
# so CPack doesn't try to create system directories like /app during packaging.
set(CPACK_SET_DESTDIR ON)
# Ensure packaging uses a root install prefix so archives don't include
# the default CMake install prefix (/usr/local) in their internal paths.
# This makes the zip contents start at lib/, bin/, etc. instead of
# usr/local/lib, usr/local/bin.
set(CPACK_INSTALL_PREFIX "/")
include(CPack)