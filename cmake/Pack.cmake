set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR})
add_custom_target(
    pack
    COMMAND ${CMAKE_COMMAND} --install .
    COMMAND tar -cf ${PKG_NAME}.tar.gz ${PKG_NAME}
    COMMAND md5sum ${PKG_NAME}.tar.gz > ${PKG_NAME}.md5sum
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    DEPENDS ${PROJECT_NAME}
    COMMENT "Packing the ${PKG_NAME}"
)