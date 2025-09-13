extends MeshInstance

func _ready():
	
	set_surface_material(0,get_surface_material(0).duplicate())
	get_surface_material(0).set_shader_param("scale",scale)
