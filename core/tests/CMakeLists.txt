include(FetchContent)
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.8.1
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(Catch2)

add_executable(starsim_core_tests
    test_math.cpp
    test_utc_date_time.cpp
    test_geodetic.cpp
    test_spice_time.cpp
    test_ephemeris_validation.cpp
    test_orientation_validation.cpp
    test_integrators.cpp
    test_kepler.cpp
    test_orbit_propagation.cpp
    test_artemis_validation.cpp
    test_eclipse.cpp
    test_vehicle_models.cpp
    test_flight_body.cpp
    test_ift3_validation.cpp
    test_flight_session.cpp
)
target_link_libraries(starsim_core_tests PRIVATE starsim::core Catch2::Catch2WithMain)
target_compile_definitions(starsim_core_tests PRIVATE
    STARSIM_KERNEL_DIR="${STARSIM_DATA_DIR}/spice"
    STARSIM_DATA_DIR="${STARSIM_DATA_DIR}"
    STARSIM_OUTPUT_DIR="${CMAKE_BINARY_DIR}"
    STARSIM_CONFIG_DIR="${CMAKE_SOURCE_DIR}/config"
    STARSIM_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/validation/fixtures"
)

include(Catch)
catch_discover_tests(starsim_core_tests)
