extends RigidBody3D

# Flags (set these as desired in the Inspector)
var centers = {
	"mass": Vector3(0, 0, 0),
	"pressure": Vector3(0, 0, -1),
	"thrust": Vector3(0, 0, -2)
}

var properties = {
	"mass": 150.0,
	"total_lift": 28.0,
}

var FORCE = 50.0

# Internal state
var smoking: bool = false
var player_cam = null
var tracking: bool = true
var speed:   float = 0.0
var cam_pos: Vector3 = Vector3.ZERO

# PID controllers for yaw and pitch
@onready var pidx0 = PID.new()
@onready var pidy0 = PID.new()
@onready var adv_move = ADV_MOVE.new()
@onready var particles = gpu_particle_effects.new()

var grav: Vector3 = Vector3.ZERO

func _ready() -> void:
	freeze = false
	gravity_scale = 0.0
	linear_damp = 0.75
	angular_damp = 0.5
	mass = max(1.0, properties["mass"])
	inertia = Vector3(3, 3, 3) * mass
	center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	center_of_mass = centers["mass"]
	grav = Vector3.DOWN * 9.80665 * mass

var prev_vel: Vector3 = Vector3.ZERO
var vec: Vector3 = Vector3.ZERO
func _physics_process(_delta: float) -> void:
	if player_cam == null:
		player_cam = get_tree().root.get_child(0).get_node("Player/Player_Camera")
	else:
		cam_pos = adv_move.get_offset_position(global_position, global_basis, Vector3(0, 5, 0))
	
	# Godot forward is -Z
	var FORWARD = -global_transform.basis.z
	speed = linear_velocity.length()
	
	var throttle = Input.get_action_raw_strength("throttle_p") * -0.5 \
				 + Input.get_action_raw_strength("throttle_n") *  0.5 + 0.5
	apply_force(FORWARD * throttle * mass * FORCE, centers["thrust"])
	
	if !smoking:
		smoking = true
		add_child(particles.smoke_01())
	
	# apply_central_force(grav)  # uncomment if you want gravity
	
	var A = 0.0
	var B = 0.0
	var C = 100.0
	
	if speed > 0.1:
		var vel_dir = linear_velocity.normalized()
		
		# Basis.looking_at is static → no linter warning
		var desired_basis = Basis.looking_at(vel_dir, Vector3.UP)
		var desired_quat = Quaternion(desired_basis)
		var current_quat = Quaternion(global_transform.basis)
		var delta_quat = desired_quat * current_quat.inverse()
		
		var align_axis = delta_quat.get_axis()
		var align_angle = delta_quat.get_angle()
		
		apply_force(align_axis.normalized() * align_angle * speed * A * mass, centers["pressure"])
		
		var cur_accel = linear_velocity - prev_vel
		var anti_drag = -cur_accel * 1.25 * ((A+B)/2.0)
		apply_force(anti_drag * FORWARD * clamp(A, 0, 1), centers["mass"])
	
	if player_cam:
		var aim_dir = -player_cam.global_transform.basis.z
		var target_position = (player_cam.global_position + aim_dir * 300.0).normalized()
		
		var steer = adv_move.torque_to_position(global_transform, -Vector3.FORWARD, target_position) * mass
	
		if speed > 0.1:
			var vel_dir = linear_velocity.normalized()
			var vel_basis = Basis.looking_at(vel_dir, Vector3.UP)
			var vel_quat = Quaternion(vel_basis)
			var body_quat = Quaternion(global_transform.basis)
			var delta_vel_q = vel_quat * body_quat.inverse()
			vec = delta_vel_q.get_axis().normalized() * delta_vel_q.get_angle() * B * mass
		
		apply_torque(steer * C + vec)
	
	prev_vel = linear_velocity
