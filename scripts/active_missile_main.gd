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
	angular_damp = 3.0
	mass = max(1.0, properties["mass"])
	inertia = Vector3(3, 3, 3) * mass
	center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	center_of_mass = centers["mass"]
	grav = Vector3.DOWN * 9.80665 * mass

var prev_vel: Vector3 = Vector3.ZERO
var vec: Vector3 = Vector3.ZERO
func _physics_process(delta: float) -> void:
	if player_cam == null:
		player_cam = get_tree().root.get_child(0).get_node("Player/Player_Camera")
	else:
		cam_pos = adv_move.get_offset_position(global_position, global_basis, Vector3(0, 5, 0))
	
	# Godot forward is -Z
	var FORWARD = basis.z
	speed = linear_velocity.length()
	
	var throttle = Input.get_action_raw_strength("throttle_p") * -0.5 \
				 + Input.get_action_raw_strength("throttle_n") *  0.5 + 0.5
	apply_force(FORWARD * throttle * mass * FORCE, centers["thrust"])
	
	if !smoking:
		smoking = true
		add_child(particles.smoke_01())
	
	apply_central_force(grav)
	
	var A = 0.1
	var B = 0.1
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
	
	if player_cam:
		var aim_dir = -player_cam.global_transform.basis.z
		var target_position = (player_cam.global_position + aim_dir * 1000.0)
		
		var steer = get_alignment_torque(target_position) * mass * C
	
		if speed > 0.1:
			var vel_dir = linear_velocity.normalized()
			var vel_basis = Basis.looking_at(vel_dir, Vector3.UP)
			var vel_quat = Quaternion(vel_basis)
			var body_quat = Quaternion(global_transform.basis)
			var delta_vel_q = vel_quat * body_quat.inverse()
			vec = delta_vel_q.get_axis().normalized() * delta_vel_q.get_angle() * B * mass
		
		apply_torque(steer + vec)
	
	prev_vel = linear_velocity

func get_alignment_torque(target: Vector3) -> Vector3:
	# Desired world-space direction (unit)
	var to_target: Vector3 = (target - global_transform.origin).normalized()
	
	# Current world-space direction of the chosen local vector (unit)
	var v_current: Vector3 = (global_transform.basis * Vector3.BACK).normalized()
	
	# Angle error
	var dot_val: float = clamp(v_current.dot(to_target), -1.0, 1.0)
	var angle_err: float = acos(dot_val)            # radians  (0‥π)
	if angle_err < 1e-5:
		return Vector3.ZERO                         # already aligned
	
	# Rotation axis (handle the 180° anti-parallel case)
	var axis: Vector3 = v_current.cross(to_target)
	if axis.length_squared() < 1e-8:                # vectors are opposite
		axis = v_current.cross(Vector3.RIGHT)
		if axis.length_squared() < 1e-8:
			axis = v_current.cross(Vector3.UP)
	axis = axis.normalized()
	
	# Proportional term (τ = k_p · θ · axis)
	var torque_p: Vector3 = axis * angle_err * 12.0
	
	# Derivative term (damp current spin around *any* axis)
	var torque_d: Vector3 = -angular_velocity * 2.0
	
	return torque_p + torque_d

func roll_pd(target_roll: float, kp: float = 12.0, kd: float = 3.0) -> Vector3:
	var f = global_transform.basis.z.normalized()
	var r = f.cross(Vector3.UP)
	if r.length_squared() < 1e-4:
		r = f.cross(Vector3.RIGHT)
	r = r.normalized()
	var u = r.cross(f)

	var cur = atan2(global_transform.basis.y.dot(r),global_transform.basis.y.dot(u))
	var err := wrapf(target_roll - cur, -PI, PI)

	var torque = f * (err * kp - angular_velocity.dot(f) * kd)
	return torque
