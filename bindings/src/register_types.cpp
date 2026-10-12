#include <gdextension_interface.h>

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "flight_sim.hpp"
#include "starsim_core.hpp"

namespace {

void initialize_starsim(godot::ModuleInitializationLevel level) {
    if (level != godot::MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
    GDREGISTER_CLASS(starsim::bindings::StarsimCore);
    GDREGISTER_CLASS(starsim::bindings::FlightSim);
}

void uninitialize_starsim(godot::ModuleInitializationLevel) {}

}  // namespace

extern "C" {

GDExtensionBool GDE_EXPORT starsim_library_init(GDExtensionInterfaceGetProcAddress get_proc_address,
                                                GDExtensionClassLibraryPtr library,
                                                GDExtensionInitialization* initialization) {
    godot::GDExtensionBinding::InitObject init_obj(get_proc_address, library, initialization);
    init_obj.register_initializer(initialize_starsim);
    init_obj.register_terminator(uninitialize_starsim);
    init_obj.set_minimum_library_initialization_level(godot::MODULE_INITIALIZATION_LEVEL_SCENE);
    return init_obj.init();
}

}
