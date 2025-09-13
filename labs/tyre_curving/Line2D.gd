tool
extends Node2D

export var grip: float = 10 setget Xgrip
func Xgrip(val):
	grip = val
	refresh()
	
export var stiffness: float = 1 setget Xstiffness
func Xstiffness(val):
	stiffness = val
	refresh()

export(float,0.001,5) var peak_y: float = 1 setget Xpeak_y
func Xpeak_y(val):
	peak_y = val
	refresh()

export(float,0.001,5) var peak_x: float = 1 setget Xpeak_x
func Xpeak_x(val):
	peak_x = val
	refresh()

export(float,0,1) var linear: float setget Xlinear
func Xlinear(val):
	linear = val
	refresh()

export(float,0,1) var aspect_ratio: float = 1 setget Xaspect_ratio
func Xaspect_ratio(val):
	aspect_ratio = val
	refresh()

export(float,0,1) var shape_x: float = 1 setget Xshape_x
func Xshape_x(val):
	shape_x = val
	refresh()

export(float,0,1) var shape_y: float = 1 setget Xshape_y
func Xshape_y(val):
	shape_y = val
	refresh()

export(float,0,1) var TEST_conflict: float setget XTEST_conflict
func XTEST_conflict(val):
	TEST_conflict = val
	refresh()

export var TEST_resolution: int = 100 setget XTEST_resolution
func XTEST_resolution(val):
	TEST_resolution = val
	refresh()


func refresh():
	$graph_y.clear_points()
	$linear_graph_y.clear_points()
	$graph_x.clear_points()
	$linear_graph_x.clear_points()
	
	for i in range(TEST_resolution):
		
		var disty: float = i*stiffness/100.0
		var distx: float = i*stiffness*TEST_conflict/100.0
		
		var slip: float = max(Vector2(distx,disty).length()/(grip*(peak_y*(1.0-linear) +linear)) -1.0,0)
		var peaked: float = 1.0 -(1.0/(max(Vector2(distx,disty).length()/(grip*peak_y)/peak_y -1,0) +1.0))
		
		var forcey: float = disty/(slip*((peaked/shape_y)*peak_y*(1.0-linear) + linear) +1)
		
		$graph_y.add_point(Vector2(i/100.0,-forcey)*1000.0)
	
	for i in range(TEST_resolution):
		
		var disty: float = i*stiffness/100.0
		var distx: float = i*TEST_conflict/stiffness/100.0
		
		var slip: float = max(Vector2(distx,disty).length()/grip -1.0,0)
		
		var forcey: float = disty/(slip +1)
		
		$linear_graph_y.add_point(Vector2(i/100.0,-forcey)*1000.0)

	for i in range(TEST_resolution):
		
		var disty: float = i*stiffness*aspect_ratio/100.0
		var distx: float = i*TEST_conflict/stiffness/100.0
		
		var slip: float = max(Vector2(distx,disty).length()/(grip*(peak_x*(1.0-linear) +linear)) -1.0,0)
		var peaked: float = 1.0 -(1.0/(max(Vector2(distx,disty).length()/(grip*peak_x)/peak_x -1,0) +1.0))
		
		var forcey: float = disty/(slip*((peaked/shape_x)*peak_x*(1.0-linear) + linear) +1)
		
		$graph_x.add_point(Vector2(i/100.0,-forcey)*1000.0)
	
	for i in range(TEST_resolution):
		
		var disty: float = i*stiffness*aspect_ratio/100.0
		var distx: float = i*TEST_conflict/stiffness/100.0
		
		var slip: float = max(Vector2(distx,disty).length()/grip -1.0,0)
		
		var forcey: float = disty/(slip +1)
		
		$linear_graph_x.add_point(Vector2(i/100.0,-forcey)*1000.0)
