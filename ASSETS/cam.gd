extends Position3D

export var car: NodePath
export var enabled: bool
export var self_align: bool = true


func _physics_process(delta):
	if enabled:
		var c: RigidBody = get_node(car)
		if self_align:
			look_at(c.global_translation,Vector3(0,1,0))
		global_translation = c.global_translation
		translate_object_local(Vector3(0,0,4.5))
		$duck.translation.y = c.OUTPUT_total_compressed
