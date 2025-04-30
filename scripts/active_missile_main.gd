extends RigidBody3D

# Flags (set these as desired in the Inspector)
var centers = {
	"mass": Vector3(0,0,0),
	"pressure": Vector3(0,0,-1),
	"thrust": Vector3(0,0,-2)
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
var speed: float = 0.0
var cam_pos: Vector3 = Vector3.ZERO

# PID controllers for yaw and pitch
@onready var pidx0 = PID.new()
@onready var pidy0 = PID.new()
@onready var adv_move = ADV_MOVE.new()
@onready var particles = gpu_particle_effects.new()

var grav: Vector3 = Vector3.ZERO
func _ready() -> void:
	
	# Basic physics settings
	freeze = false
	gravity_scale = 0.0
	linear_damp = 0.75
	angular_damp = 3.0
	mass = max(1.0, properties["mass"])
	inertia = Vector3(3, 3, 3) * mass
	center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	center_of_mass = centers["mass"]
	grav = (Vector3.DOWN * 9.80665 * mass)

var prev_vel: Vector3 = Vector3.ZERO
func _physics_process(_delta: float) -> void:
	if player_cam == null:
		player_cam = get_tree().root.get_child(0).get_node("Player/Player_Camera")
	else:
		cam_pos = adv_move.get_offset_position(global_position, global_basis, Vector3(0,5,0))
		
	# Godot forward is -Z; using that here guarantees consistent math
	var FORWARD = -global_transform.basis.z
	speed = linear_velocity.length()
	
	# Thrust: Apply force along forward direction.
	var throttle_a = Input.get_action_raw_strength("throttle_p")
	var throttle_b = Input.get_action_raw_strength("throttle_n")
	var throttle = (throttle_a/2.0 - 1*throttle_a) + (throttle_b/2.0) + 0.5
	var pos = adv_move.get_offset_position(global_position, global_basis, centers["thrust"])
	apply_force(FORWARD * throttle * mass * FORCE, pos)
	# Optionally trigger smoke effect here.
	if not smoking:
		smoking = true
		add_child(particles.smoke_01())
	
	# Gravity (handled manually so we can still set gravity_scale = 0)
	#apply_central_force(grav)
	
	var A = 0.0
	var B = 0.0
	var C = 10.0
	
	# Apply aerodynamic alignment (forward flight toward missile's forward direction)
	if speed > 0.1:
		var vel_dir = linear_velocity.normalized()
		
		# --- Quaternion-based alignment force (no Euler angles) ---
		var desired_basis = Basis().looking_at(vel_dir, Vector3.UP)    # velocity direction
		var desired_quat  = Quaternion(desired_basis)
		var current_quat  = Quaternion(global_transform.basis)
		var delta_quat    = desired_quat * current_quat.inverse()
		
		var align_axis  = delta_quat.get_axis()
		var align_angle = delta_quat.get_angle()
		
		# alignment force applied at centre of pressure
		var afd = align_axis.normalized()
		var afm = align_angle
		apply_force(afd * afm * speed * A * mass, centers["pressure"])
	
		# counteract unwanted de-acceleration forces from alignment forces
		var cur_accel = linear_velocity - prev_vel
		var anti_drag = -cur_accel * 1.25 * (A+B)/2
		apply_force(anti_drag * FORWARD * clamp(A,0,1), centers["mass"])
		
	# --- Guidance torque -------------------------------------------------
	if player_cam:
		# Desired aim point 300 m ahead of the camera
		var player_cam_aim_dir = -player_cam.global_transform.basis.z
		var target_position = player_cam.global_transform.origin + player_cam_aim_dir * 300.0
		
		var steer = orient_transform(global_transform, Vector3.UP, target_position) * mass
		
		# Optional extra aerodynamic damping torque (B is zero by default)
		if speed > 0.1:
			var vel_dir = linear_velocity.normalized()
			var desired_basis = Basis().looking_at(vel_dir, Vector3.UP)
			var desired_quat  = Quaternion(desired_basis)
			var current_quat  = Quaternion(global_transform.basis)
			var delta_quat    = desired_quat * current_quat.inverse()
			var axis_align    = delta_quat.get_axis()
			var angle_align   = delta_quat.get_angle()
			steer += axis_align.normalized() * angle_align * B * mass
		
		apply_torque(steer * C)
	
	prev_vel = linear_velocity



# Rotates an object to point its forward axis at a target,
# while keeping its up axis aligned with world up. Fully avoids gimbal lock.
func orient_transform(
	xform: Transform3D,          # renamed to avoid shadowing Node3D.transform
	forward_axis: Vector3,
	target_position: Vector3
) -> Vector3:
	var to_target = (target_position - xform.origin).normalized()

	# Build desired basis manually (avoids gimbal lock entirely)
	var desired_forward = to_target
	var desired_right   = forward_axis.cross(desired_forward).normalized()
	if desired_right.length_squared() < 1e-4:
		return Vector3.ZERO               # Degenerate (looking straight up/down)
	var desired_up      = desired_forward.cross(desired_right).normalized()
	var desired_basis   = Basis(desired_right, desired_up, desired_forward)

	var current_quat  = Quaternion(xform.basis.orthonormalized())
	var desired_quat  = Quaternion(desired_basis.orthonormalized())
	var delta_quat    = desired_quat * current_quat.inverse()

	var axis  = delta_quat.get_axis().normalized()
	var angle = delta_quat.get_angle()
	var raw_steer = axis * angle

	# Constrain to yaw and pitch only
	var current_right = xform.basis.x.normalized()
	var yaw_component   = forward_axis * raw_steer.dot(forward_axis)
	var pitch_component = current_right * raw_steer.dot(current_right)
	var steer_torque    = yaw_component + pitch_component

	# Roll-level the missile (align local up with world up)
	var current_up  = xform.basis.y.normalized()
	var dot_up      = clamp(current_up.dot(Vector3.UP), -1.0, 1.0)

	var roll_axis  : Vector3
	var roll_angle : float
	if dot_up > 0.9999:
		roll_axis  = Vector3.ZERO
		roll_angle = 0.0
	elif dot_up < -0.9999:
		roll_axis  = forward_axis.normalized()
		roll_angle = PI
	else:
		roll_axis  = current_up.cross(Vector3.UP).normalized()
		roll_angle = acos(dot_up)

	var roll_torque = roll_axis * roll_angle if roll_axis.length_squared() > 1e-4 else Vector3.ZERO

	return steer_torque + roll_torque
