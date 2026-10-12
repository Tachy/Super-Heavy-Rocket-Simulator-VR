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
    src/environment/us76_atmosphere.cpp
    src/propulsion/engine_performance.cpp
    src/aero/slender_body_aero.cpp
    src/vehicle/vehicle.cpp
    src/vehicle/flight_body.cpp
    src/time/utc_date_time.cpp
    src/frames/geodetic.cpp
    src/environment/point_mass_gravity.cpp
    src/environment/eclipse.cpp
    src/orbit/kepler.cpp
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
    target_compile_options(starsim_core PRIVATE /W4 /permissive- /fp:precise)
else()
    target_compile_options(starsim_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
