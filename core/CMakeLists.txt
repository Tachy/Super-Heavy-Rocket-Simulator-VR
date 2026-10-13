include(FetchContent)
FetchContent_Declare(
    tomlplusplus
    GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
    GIT_TAG v3.4.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(tomlplusplus)

add_library(starsim_core STATIC
    src/config/vehicle_config.cpp
    src/config/launch_site_config.cpp
    src/config/mission_config.cpp
    src/environment/us76_atmosphere.cpp
    src/propulsion/engine_performance.cpp
    src/aero/slender_body_aero.cpp
    src/aero/grid_fin.cpp
    src/aero/flap.cpp
    src/aero/entry_heating.cpp
    src/vehicle/vehicle.cpp
    src/vehicle/flight_body.cpp
    src/gnc/attitude_controller.cpp
    src/gnc/ascent_guidance.cpp
    src/gnc/burn_node.cpp
    src/gnc/booster_return.cpp
    src/gnc/descent_model.cpp
    src/gnc/descent_discretization.cpp
    src/gnc/descent_problem.cpp
    src/gnc/descent_planner.cpp
    src/gnc/landing_site.cpp
    src/gnc/descent_snapshot.cpp
    src/gnc/descent_replanner.cpp
    src/gnc/descent_tracking.cpp
    src/gnc/scvx_landing.cpp
    src/gnc/ship_landing.cpp
    src/gnc/tvc_lateral.cpp
    src/gnc/attitude_slew.cpp
    src/gnc/entry_aero_table.cpp
    src/gnc/entry_attitude.cpp
    src/gnc/deorbit_planner.cpp
    src/gnc/entry_guidance.cpp
    src/gnc/entry_predictor.cpp
    src/sim/telemetry_recorder.cpp
    src/sim/catch_tower_sim.cpp
    src/sim/leg_landing_sim.cpp
    src/sim/landing_outcome.cpp
    src/sim/flight_session.cpp
    src/mission/lunar_transfer.cpp
    src/contact/primitives.cpp
    src/contact/tower_geometry.cpp
    src/contact/catch_arms.cpp
    src/contact/pin_contact.cpp
    src/contact/catch_trigger.cpp
    src/contact/tower_collision.cpp
    src/contact/landing_surface.cpp
    src/contact/landing_legs.cpp
    src/optim/sparse_matrix.cpp
    src/optim/minimum_degree.cpp
    src/optim/ldl.cpp
    src/optim/cones.cpp
    src/optim/socp_problem.cpp
    src/optim/socp_builder.cpp
    src/optim/equilibration.cpp
    src/optim/kkt_system.cpp
    src/optim/socp_solver.cpp
    src/time/utc_date_time.cpp
    src/frames/geodetic.cpp
    src/environment/point_mass_gravity.cpp
    src/environment/eclipse.cpp
    src/orbit/kepler.cpp
    src/orbit/lambert.cpp
    src/orbit/ballistic_propagator.cpp
    src/spice/spice_session.cpp
    src/spice/spice_time_converter.cpp
    src/spice/spice_ephemeris.cpp
    src/spice/spice_orientation.cpp
)
add_library(starsim::core ALIAS starsim_core)

target_include_directories(starsim_core PUBLIC include)
# CSPICE is an implementation detail of the spice adapter: never exposed in public headers.
target_link_libraries(starsim_core PRIVATE cspice tomlplusplus::tomlplusplus)

if(MSVC)
    target_compile_options(starsim_core PRIVATE /W4 /permissive- /fp:precise /utf-8)
else()
    target_compile_options(starsim_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
