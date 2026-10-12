# Builds the NAIF CSPICE toolkit from the sources fetched by tools/data_pipeline/fetch.py.
# Compiled by us (instead of using the shipped cspice.lib) so that the C runtime matches.

set(CSPICE_ROOT "${STARSIM_EXTERNAL_DIR}/cspice")
if(NOT EXISTS "${CSPICE_ROOT}/src/cspice/spkezr_c.c")
    message(FATAL_ERROR "CSPICE sources not found in ${CSPICE_ROOT}. Run: python tools/data_pipeline/fetch.py cspice")
endif()

file(GLOB CSPICE_SOURCES CONFIGURE_DEPENDS "${CSPICE_ROOT}/src/cspice/*.c")
add_library(cspice STATIC ${CSPICE_SOURCES})
target_include_directories(cspice SYSTEM PUBLIC "${CSPICE_ROOT}/include")
target_compile_definitions(cspice PRIVATE _COMPLEX_DEFINED MSDOS OMIT_BLANK_CC NON_ANSI_STDIO)
if(MSVC)
    # f2c-translated code: silence warnings; zzsecprt.c must not be optimised (see mkprodct.bat).
    target_compile_options(cspice PRIVATE /w)
    set_source_files_properties("${CSPICE_ROOT}/src/cspice/zzsecprt.c" PROPERTIES COMPILE_OPTIONS "/Od")
else()
    target_compile_options(cspice PRIVATE -w)
endif()
