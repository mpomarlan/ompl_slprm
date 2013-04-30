# The default should be "ON" for contributions. Since this sample
# contribution doesn't do anything useful it's disabled by default.
option(OMPL_CONTRIB_SLPRM_CONTRIB "Enable SLPRM contrib" ON)

if(OMPL_CONTRIB_SLPRM_CONTRIB)
    file(GLOB_RECURSE OMPL_CONTRIB_SLPRM_CONTRIB_SOURCE_CODE src/*.cpp)
    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_CONTRIB_SLPRM_CONTRIB_SOURCE_CODE} PARENT_SCOPE)

    if(OMPL_BUILD_TESTS)
# TODO: define test files; the one below just shows the CMake file syntax
#        add_ompl_test(test_spars tests/test_debug_spars.cpp)
    endif()

endif()
