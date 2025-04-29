extends RigidBody3D

# Flags (set these as desired in the Inspector)
var centers = {
	"mass": Vector3(0,0,1),
	"pressure": Vector3(0,0,0),
	"thrust": Vector3(0,0,-3)
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
	linear_damp = 0.1
	angular_damp = 5.0
	mass = max(1.0, properties["mass"])
	inertia = Vector3(1, 1, 1) * mass
	center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	center_of_mass = centers["mass"]
	grav = (Vector3.DOWN * 9.80665 * mass)

var prev_vel: Vector3 = Vector3.ZERO
func _physics_process(delta: float) -> void:
	if player_cam == null:
		player_cam = get_tree().root.get_child(0).get_node("Player/Player_Camera")
	else:
		cam_pos = adv_move.get_offset_position(global_position, global_basis, Vector3(0,0,-10))
		
	var FORWARD = global_transform.basis.z
	speed = max(1, linear_velocity.dot(global_transform.basis.z))
	
	# Thrust: Apply force along forward direction if within fuel duration.
	var throttle_a = Input.get_action_raw_strength("throttle_p")
	var throttle_b = Input.get_action_raw_strength("throttle_n")
	var throttle = (throttle_a/2.0 - 1*throttle_a) + (throttle_b/2.0) + 0.5
	apply_force(FORWARD * throttle * mass * FORCE, centers["thrust"])
	# Optionally trigger smoke effect here.
	if not smoking:
		smoking = true
		add_child(particles.smoke_01())
	
	# Gravity: Apply a downward force.
	apply_central_force(grav)
	
	var A = 1.0 # val > 0 = +aim -flight, val < 0 = -aim +flight
	
	# Apply aerodynamic alignment (forward flight toward missile's forward direction)
	var afd = (FORWARD - linear_velocity.normalized()).normalized()
	var afm = FORWARD.angle_to(linear_velocity.normalized())
	var e1 = clamp(-A+1, 0, 1)
	apply_force(afd * afm * speed * e1 * mass, centers["pressure"])
	
	# counteract unwanted de-acceleration forces from alignment forces
	var cur_accel = linear_velocity - prev_vel
	var anti_drag = -cur_accel * 1.25
	apply_force(anti_drag * FORWARD * clamp(A,0,1), centers["mass"])
	
	# apply aerodynamic alignment (missile toward foward flight)
	var axis = FORWARD.cross(linear_velocity.normalized())
	var angle = FORWARD.angle_to(linear_velocity.normalized())
	if axis.length() > 0.005 and angle > 0.005:
		var torque = axis.normalized() * angle
		var e2 = clamp(A, 0, 1)
		apply_torque(torque * speed * e2 * mass)
	
	if player_cam:
		var player_cam_aim_dir = -player_cam.global_transform.basis.z
		var target_position = player_cam.global_transform.origin + player_cam_aim_dir * 1000.0
		var vec = adv_move.torque_to_position(delta, self, Vector3.BACK, target_position)
		apply_torque(vec * mass * speed)
	
	prev_vel = linear_velocity
