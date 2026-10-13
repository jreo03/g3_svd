extends CollisionShape

export var i_scale: float = 1.0

func _ready():
	
	var new: PoolVector3Array
	
	for i in shape.get_points():
		new.append(i*i_scale)
	
	shape.set_points(new)
