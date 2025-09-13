#tool
extends Viewport

export var target_mat: ShaderMaterial


func _process(delta):
	size = get_node("..").get_viewport().size/2.0
	$Camera.global_transform = get_node("..").get_viewport().get_camera().global_transform
	$Camera.fov = get_node("..").get_viewport().get_camera().fov
