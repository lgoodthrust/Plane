extends Node3D

@export var player_scene: PackedScene
@export var plane_scene: PackedScene
var player_instance: Node
var plane_instance: Node

func _ready() -> void:
	load_plane()
	load_player()

func _process(_delta) -> void:
	if Input.is_action_just_pressed("key_alt_f4"):
		get_tree().quit()

func load_player():
	if player_scene:
		player_instance = player_scene.instantiate()
		add_child(player_instance)
		player_instance.global_position = Vector3(0, 3, 0)
		player_instance.owner = self
		print("Player loaded successfully!")
	else:
		print("Error: Player scene not assigned!")

func load_plane():
	if plane_scene:
		plane_instance = plane_scene.instantiate()
		add_child(plane_instance)
		plane_instance.global_position = Vector3(0, 5, -10)
		plane_instance.owner = self
		print("Planer loaded successfully!")
	else:
		print("Error: Plane scene not assigned!")
