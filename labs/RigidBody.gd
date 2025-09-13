extends RigidBody

export var artificial: bool

func _physics_process(delta):
	if artificial:
		rotation.x += 1/60.0
	else:
		angular_velocity.x = 1
	print([linear_velocity.length()])
