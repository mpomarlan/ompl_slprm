# This file will be configured to contain variables for CPack. These variables
# should be set in the CMake list file of the project before CPack module is
# included. Example variables are:
#   CPACK_GENERATOR                     - Generator used to create package
#   CPACK_INSTALL_CMAKE_PROJECTS        - For each project (path, name, component)
#   CPACK_CMAKE_GENERATOR               - CMake Generator used for the projects
#   CPACK_INSTALL_COMMANDS              - Extra commands to install components
#   CPACK_INSTALLED_DIRECTORIES           - Extra directories to install
#   CPACK_PACKAGE_DESCRIPTION_FILE      - Description file for the package
#   CPACK_PACKAGE_DESCRIPTION_SUMMARY   - Summary of the package
#   CPACK_PACKAGE_EXECUTABLES           - List of pairs of executables and labels
#   CPACK_PACKAGE_FILE_NAME             - Name of the package generated
#   CPACK_PACKAGE_ICON                  - Icon used for the package
#   CPACK_PACKAGE_INSTALL_DIRECTORY     - Name of directory for the installer
#   CPACK_PACKAGE_NAME                  - Package project name
#   CPACK_PACKAGE_VENDOR                - Package project vendor
#   CPACK_PACKAGE_VERSION               - Package project version
#   CPACK_PACKAGE_VERSION_MAJOR         - Package project version (major)
#   CPACK_PACKAGE_VERSION_MINOR         - Package project version (minor)
#   CPACK_PACKAGE_VERSION_PATCH         - Package project version (patch)

# There are certain generator specific ones

# NSIS Generator:
#   CPACK_PACKAGE_INSTALL_REGISTRY_KEY  - Name of the registry key for the installer
#   CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS - Extra commands used during uninstall
#   CPACK_NSIS_EXTRA_INSTALL_COMMANDS   - Extra commands used during install


SET(CPACK_BINARY_BUNDLE "")
SET(CPACK_BINARY_CYGWIN "")
SET(CPACK_BINARY_DEB "")
SET(CPACK_BINARY_DRAGNDROP "")
SET(CPACK_BINARY_NSIS "")
SET(CPACK_BINARY_OSXX11 "")
SET(CPACK_BINARY_PACKAGEMAKER "")
SET(CPACK_BINARY_RPM "")
SET(CPACK_BINARY_STGZ "")
SET(CPACK_BINARY_TBZ2 "")
SET(CPACK_BINARY_TGZ "")
SET(CPACK_BINARY_TZ "")
SET(CPACK_BINARY_ZIP "")
SET(CPACK_CMAKE_GENERATOR "Unix Makefiles")
SET(CPACK_COMPONENTS_ALL "Unspecified;pkgconfig")
SET(CPACK_COMPONENT_UNSPECIFIED_HIDDEN "TRUE")
SET(CPACK_COMPONENT_UNSPECIFIED_REQUIRED "TRUE")
SET(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "python, libboost-all-dev, libode-dev")
SET(CPACK_GENERATOR "DEB;TGZ")
SET(CPACK_INSTALL_CMAKE_PROJECTS "/wg/stor5/mpomarlan/ompl/build;ompl;ALL;/")
SET(CPACK_INSTALL_PREFIX "/opt/ros/groovy")
SET(CPACK_MODULE_PATH "/wg/stor5/mpomarlan/ompl/CMakeModules")
SET(CPACK_NSIS_DISPLAY_NAME "ompl 0.12.2")
SET(CPACK_NSIS_INSTALLER_ICON_CODE "")
SET(CPACK_NSIS_INSTALLER_MUI_ICON_CODE "")
SET(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES")
SET(CPACK_NSIS_PACKAGE_NAME "ompl 0.12.2")
SET(CPACK_OUTPUT_CONFIG_FILE "/wg/stor5/mpomarlan/ompl/build/CPackConfig.cmake")
SET(CPACK_PACKAGE_CONTACT "Mark Moll <mmoll@rice.edu>")
SET(CPACK_PACKAGE_DEFAULT_LOCATION "/")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "/usr/share/cmake-2.8/Templates/CPack.GenericDescription.txt")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Open Motion Planning Library (OMPL)")
SET(CPACK_PACKAGE_FILE_NAME "omplapp_0.12.2_amd64-Ubuntu12.04")
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "ompl 0.12.2")
SET(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "ompl 0.12.2")
SET(CPACK_PACKAGE_NAME "ompl")
SET(CPACK_PACKAGE_RELOCATABLE "true")
SET(CPACK_PACKAGE_VENDOR "Rice University")
SET(CPACK_PACKAGE_VERSION "0.12.2")
SET(CPACK_PACKAGE_VERSION_MAJOR "0")
SET(CPACK_PACKAGE_VERSION_MINOR "12")
SET(CPACK_PACKAGE_VERSION_PATCH "2")
SET(CPACK_RESOURCE_FILE_LICENSE "/usr/share/cmake-2.8/Templates/CPack.GenericLicense.txt")
SET(CPACK_RESOURCE_FILE_README "/usr/share/cmake-2.8/Templates/CPack.GenericDescription.txt")
SET(CPACK_RESOURCE_FILE_WELCOME "/usr/share/cmake-2.8/Templates/CPack.GenericWelcome.txt")
SET(CPACK_RSRC_DIR "/wg/stor5/mpomarlan/ompl/CMakeModules")
SET(CPACK_SET_DESTDIR "OFF")
SET(CPACK_SOURCE_CYGWIN "")
SET(CPACK_SOURCE_GENERATOR "TGZ;ZIP")
SET(CPACK_SOURCE_IGNORE_FILES "/.hg;/build/;.pyc$;.pyo$;__pycache__;.so$;.dylib$;.orig$;.DS_Store;.tm_properties;mkwebdocs.sh;/html/;/bindings/;TODO;exposed_decl.pypp.txt;ompl.pc$;installPyPlusPlus.bat$;installPyPlusPlus.sh$;create_symlinks.sh$;uninstall_symlinks.sh$;config.h$;.registered$;download.md$;mainpage.md$;binding_generator.py$")
SET(CPACK_SOURCE_OUTPUT_CONFIG_FILE "/wg/stor5/mpomarlan/ompl/build/CPackSourceConfig.cmake")
SET(CPACK_SOURCE_TBZ2 "")
SET(CPACK_SOURCE_TGZ "")
SET(CPACK_SOURCE_TZ "")
SET(CPACK_SOURCE_ZIP "")
SET(CPACK_SYSTEM_NAME "Linux")
SET(CPACK_TOPLEVEL_TAG "Linux")
