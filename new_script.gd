# Minimal “drop-in” helper for any RigidBody3D (Godot 4.x)
# Call roll_pd(target_bank_rad, delta) from _physics_process().
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
