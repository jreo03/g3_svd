extends Camera


var ok_pos: Vector3
var ok_vel: Vector3
export var fov_base: float = 200

func _physics_process(delta):
	
	var _car: SVD_BODY = $"../car"
	
	var dist: float = _car.global_translation.distance_to(global_translation)
	if dist>100:
		global_translation = _car.global_translation +Vector3.UP*4 + _car.linear_velocity.normalized()*Vector3(99,0,99)
	
	ok_vel = lerp(ok_vel,_car.linear_velocity*delta/0.05,0.1)
	
	var ref: Vector3 = _car.global_translation + ok_vel
	
	
	fov = clamp(fov_base/global_translation.distance_to(_car.global_translation),1,70)
	
	ok_pos -= (ok_pos - ref)*0.05
	
	look_at(ok_pos,Vector3.UP)
	
