tool
extends Node2D

export var load_sens: float = 1 setget xload_sens
func xload_sens(val):
	load_sens = val
	refresh()

export var max_load: float = 100 setget xmax_load
func xmax_load(val):
	max_load = val
	refresh()

export var TEST_resolution: int = 100

func refresh():
	$graph.clear_points()
	$linear_graph.clear_points()
	
	for i in range(TEST_resolution):
		var progressed: float = i/float(TEST_resolution -1)
		
		var _load: float = progressed*max_load
		var grip: float = _load/(_load*load_sens/100.0 +1.0)
		
		$graph.add_point(Vector2(progressed*100.0, -grip/max_load*100.0)*10.0)
		$linear_graph.add_point(Vector2(progressed*100.0, -_load/max_load*100.0)*10.0)
