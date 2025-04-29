extends Control

@onready var ftps: Label = $FTPS
@onready var tps: Label = $TPS
@onready var fps: Label = $FPS

# Declare necessary variables
var camera: Camera3D
var target: Node
var msls
var buildering: bool = false

var FOV

var w_size = DisplayServer.window_get_size()
var v_size: Vector2 = DisplayServer.screen_get_size()
var ar: float = v_size.x / v_size.y  # Aspect ratio (width/height)
var w_center = w_size / 2.0
var world: Node

# Called when the node enters the scene tree for the first time
func _ready() -> void:
	world = get_tree().current_scene.get_node_or_null(".")
	print("center: ", w_center)
	camera = get_parent()  # Assuming the parent is the camera node
	if not camera:
		print("Warning: Parent is not a Camera3D!")
	FOV = camera.fov

func _physics_process(_delta: float) -> void:
	if ftps:
		ftps.text = "FTPS: " + str(1/_delta)

# Process function to control redrawing
func _process(_delta: float) -> void:
	if tps and fps:
		tps.text = "TPS: " + str(1/_delta)
		fps.text = "FPS: " + str(Engine.get_frames_per_second())
