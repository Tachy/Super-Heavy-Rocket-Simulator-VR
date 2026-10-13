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
    test_flight_session.cpp
    test_ascent_guidance.cpp
    test_lunar_transfer.cpp
    test_booster_return.cpp
    test_descent_telemetry.cpp
    test_grid_fins.cpp
    test_flaps.cpp
    test_entry_heating.cpp
    test_optim_sparse.cpp
    test_optim_ordering.cpp
    test_optim_ldl.cpp
    test_optim_cones.cpp
    test_optim_builder.cpp
    test_optim_equilibration.cpp
    test_optim_kkt.cpp
    test_optim_solver.cpp
    test_optim_validation.cpp
    test_launch_site_config.cpp
    test_contact_primitives.cpp
    test_tower_geometry.cpp
    test_catch_arms.cpp
    test_pin_contact.cpp
    test_catch_trigger.cpp
    test_catch_tower.cpp
    test_socp_reuse.cpp
    test_descent_model.cpp
    test_descent_discretization.cpp
    test_descent_problem.cpp
    test_descent_planner.cpp
    test_landing_outcome.cpp
    test_descent_snapshot.cpp
    test_descent_replanner.cpp
    test_descent_tracking.cpp
    test_scvx_landing.cpp
    test_scvx_catch.cpp
    test_attitude_bench.cpp
    test_tvc_lateral.cpp
)
target_link_libraries(starsim_core_tests PRIVATE starsim::core Catch2::Catch2WithMain)
target_compile_definitions(starsim_core_tests PRIVATE
    STARSIM_KERNEL_DIR="${STARSIM_DATA_DIR}/spice"
    STARSIM_DATA_DIR="${STARSIM_DATA_DIR}"
    STARSIM_OUTPUT_DIR="${CMAKE_BINARY_DIR}"
    STARSIM_CONFIG_DIR="${CMAKE_SOURCE_DIR}/config"
    STARSIM_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/validation/fixtures"
    STARSIM_SOCP_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/validation/socp"
    STARSIM_TELEMETRY_DIR="${CMAKE_SOURCE_DIR}/tests/validation/telemetry"
)

include(Catch)
catch_discover_tests(starsim_core_tests)
