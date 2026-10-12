class_name Paths
## Locations of repository folders outside the Godot project (data/ and config/ are siblings of game/).


static func repo_dir() -> String:
	return ProjectSettings.globalize_path("res://").path_join("..").simplify_path()


static func data_dir() -> String:
	return repo_dir().path_join("data")


static func config_dir() -> String:
	return repo_dir().path_join("config")
