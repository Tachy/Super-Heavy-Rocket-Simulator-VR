class_name I18nEvents
extends RefCounted
## The change notification of the language service (I18n.events); a RefCounted so the static I18n class can own it.

signal language_changed(language: String)
