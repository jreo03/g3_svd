extends Spatial

onready var car: SVD_BODY = get_parent()
export var disabled: bool

func _process(delta):
	if disabled:
		$enoff.stop()
		$enon.stop()
		return
	
	$enon.unit_db = max(linear2db(car.throttle),-80.0)
	$enoff.unit_db = max(linear2db(1.0 -car.throttle),-80.0)
	$enon.max_db = $enon.unit_db
	$enoff.max_db = $enoff.unit_db
	
	$enon.pitch_scale = max(car.rpm,800)/8000.0
	$enoff.pitch_scale = max(car.rpm,800)/8000.0
