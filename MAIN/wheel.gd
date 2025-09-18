extends RayCast
class_name SVD_WHEEL
export var DEBUG_testspin: float = 100
export var SUS_spring_stiffness: float = 40
export var SUS_dampening: float = 4.5
export var SUS_rest_length: float = 0.3
export var WL_size: float = 0.55
export var WL_weight: float = 1.0
export var TR_stiffness: float = 40
export var TR_deform_factor: float = 1
export var TR_deform_threshold: float = 1
export var TR_spin_resistence_rate: float = 20
export var TR_friction: float = 1
export var TR_height: float = 0.1
export var TR_soft_tyre: bool
export var TR_elasticity: float = 2500
export var TR_dampening: float = 25

export(float,0.001,1) var TR_MDL_peak_y: float = 1 # pacejka = 1 # default = 0.748
export(float,0.001,1) var TR_MDL_peak_x: float = 0.936 # pacejka = 0.936 # default = 0.748
export(float,0.001,1) var TR_MDL_aspect_ratio: float = 1 # pacejka = 0.228 # default = 1
export(float,0,1) var TR_MDL_shape_x: float = 0.855 # pacejka = 0.855 # default = 1
export(float,0,1) var TR_MDL_shape_y: float = 0.975 # pacejka = 0.975 # default = 1


export(float,0,1) var TR_MDL_linear: float = 0

export var DT_influence: float = 1
export var DT_BrakeTorque: float = 5
export var DT_HandbrakeBias: float = 0
export var DT_BrakeBias: float = 1
export(int, "Bump", "Absorb") var curb_step_behaviour = 1

export(Array, NodePath) var AN_spinning_meshes: Array
export(Array, NodePath) var AN_fixed_meshes: Array
#export(Array, NodePath) var AN_steering_meshes: Array
var a_s_m: Array
var a_f_m: Array
#var a_s2_m: Array

onready var car: RigidBody = get_parent()

var spin: float = 0
var total_w_weight: float
var prev_compressed: float
var predict_prev_compressed: float

var HUB_pos: float
var HUB_velocity: float
var HUB_thrust: float
var HUB_inertia_thrust: float
var HUB_inertia: float
var HUB_past_pos: float
export var HUB_max_travel: float = 0.3
export var HUB_min_travel: float = -10.1
export var HUB_spring_rest: float = 0.45
onready var AN_hub: Position3D = $hub
onready var AN_spin: Position3D = $hub/spin
onready var AN_steer: Position3D = $hub/steer
onready var AN_fixed: Position3D = $hub/fixed

var TYRE_deflated: float
var TYRE_vertical_v: float
var TYRE_past_deflated: float
export var TYRE_elasticity: float = 50.0
export var TYRE_damp: float = 50.0
export var WHEEL_weight: float = 14.0

var STATE_grounded: bool
var STATE_arm_limited: bool
var STATE_arm_limited2: bool
var STATE_brake_locked: bool
var MEASURE_aligning: float

var dt_overdrive: float
var dt_torque: float
var dt_braking: float

const newton: float = 0.0169946645619466

onready var c_v: Position3D = $c_v
onready var p_py: MeshInstance = $c_v/patch_pos_y
onready var p_px: MeshInstance = $c_v/patch_pos_x

var impulse: Array = [Vector3(),Vector3()]

var patch_pos_y: Vector2
var patch_pos_x: Vector2

var OUTPUT_skidding: float
var OUTPUT_stressing: float
var OUTPUT_grip: float
var OUTPUT_compressed: float
var past_global_axle_pos: Vector3
var predicted_s_force: float

func _ready():
	for i in AN_spinning_meshes:
		a_s_m.append(get_node(i))
	for i in AN_fixed_meshes:
		a_f_m.append(get_node(i))
#	for i in AN_steering_meshes:
#		a_s2_m.append(get_node(i))

	for i in a_s_m:
		remove_child(i)
		AN_spin.add_child(i)

	for i in a_f_m:
		remove_child(i)
		AN_fixed.add_child(i)


func _physics_process(delta):
	if not car.PSDB:
		return
	MEASURE_aligning = 0
	$c_v.visible = car.DB_forces_visible
		
	var hz_scale: float = delta*60.0
	spin += dt_torque/(dt_overdrive +1.0)
	if dt_braking>0:
		spin -= spin/max(abs(spin)/(dt_braking/(dt_overdrive +1.0)),1)
		MEASURE_aligning -= spin/max(abs(spin)/(dt_braking/(dt_overdrive +1.0)),1)
		
	var thing: float = 1.0/(dt_overdrive +1.0)
	
	var real_wheelsize: float = WL_size/2.0
	
	cast_to.y = -(SUS_rest_length + WL_size*0.5)
	
	var velocity: Vector3 = car.PSDB.get_velocity_at_local_position(global_translation - car.global_translation)
	var local_velocity: Vector3 = global_transform.basis.orthonormalized().xform_inv(velocity)
	
	if STATE_brake_locked:
		AN_spin.rotate_x(spin*delta*2.0)
	
	total_w_weight = WL_weight*1.0
	
	var c_normal: Vector3
	var c_point: Vector3 = global_translation +global_transform.basis.orthonormalized().xform(cast_to)
	var c_axis: Basis

	if is_colliding():
		var test_var: float = 1.0
		c_normal = get_collision_normal()
		c_point = get_collision_point()
		c_axis = Basis(c_normal.cross(global_transform.basis.z),c_normal,global_transform.basis.x.cross(c_normal))
		
		var world_offsetted: Vector3 = c_v.global_translation - car.global_translation
		var patch_global_velocity: Vector3 = car.PSDB.get_velocity_at_local_position(world_offsetted)
		var patch_velocity: Vector3 = c_axis.orthonormalized().xform_inv(patch_global_velocity)
		
		c_v.global_translation = c_point
		c_v.global_transform.basis = c_axis
		
		var spring_force: float
		var t_stiff_y: float = TR_elasticity
		var t_damp_y: float = TR_dampening
		var t_stiff_x: float = t_stiff_y*TR_MDL_aspect_ratio
		var t_damp_x: float = t_damp_y*TR_MDL_aspect_ratio

		# vv 2
		var deflate_gs: float
		if TR_soft_tyre:
			var predict_compressed: float = (abs(cast_to.y) - TYRE_deflated) -c_point.distance_to(global_translation)
			var predict_damp_needed: float = predict_compressed - predict_prev_compressed
			predict_prev_compressed = predict_compressed
			TYRE_vertical_v -= predict_damp_needed/delta

			var axle_velocity: Vector3 = c_axis.orthonormalized().xform_inv(global_translation - past_global_axle_pos)/delta
			var tyre_deflected: float = 0
			var deflate_offset: float = tyre_deflected*TR_height*1.0

			TYRE_vertical_v = min(axle_velocity.y,TYRE_vertical_v)
			if TYRE_deflated<0 or TYRE_deflated>TR_height:
				TYRE_vertical_v = 0

			var tyre_elast_pressurised: float = t_stiff_y*50.0
			var tyre_damp_pressurised: float = t_damp_y*100.0

			TYRE_vertical_v -= c_axis.y.y*delta*(car.PSDB.total_gravity*c_axis.y).length()
			TYRE_vertical_v += TYRE_deflated*tyre_elast_pressurised/(predicted_s_force/delta +1)
			TYRE_vertical_v -= (TYRE_vertical_v - TYRE_deflated)*min(tyre_damp_pressurised/(predicted_s_force/delta +1),1)

			TYRE_deflated -= TYRE_vertical_v*delta

			TYRE_deflated = clamp(TYRE_deflated,deflate_offset,TR_height)
	#		TYRE_deflated = 0

			deflate_gs = (TYRE_deflated - TYRE_past_deflated)
			TYRE_past_deflated = TYRE_deflated
			deflate_gs = max(deflate_gs,0)
		# end



		var compressed: float = (abs(cast_to.y) - TYRE_deflated -deflate_gs) -c_point.distance_to(global_translation)
#		if compressed<0:
#			TYRE_vertical_v = 0
#			TYRE_deflated += compressed
		if name == "fl":
			_debug.queue["compressed"] = compressed
		OUTPUT_compressed = compressed
		if curb_step_behaviour == 0:
			var damp_needed: float = compressed - prev_compressed
#			TYRE_vertical_v -= damp_needed/delta
			if name == "fl":
				_debug.queue["damp_needed"] = damp_needed
			damp_needed += deflate_gs
			spring_force = max(0.0,(compressed*1000.0)*(SUS_spring_stiffness*newton) + (SUS_dampening*1000.0)*damp_needed )
			prev_compressed = compressed
		elif curb_step_behaviour == 1:
			var arm_velocity: Vector3 = c_axis.orthonormalized().xform_inv(velocity)
			spring_force = max(0.0,(compressed*1000.0)*(SUS_spring_stiffness*newton) - (SUS_dampening*newton)*(arm_velocity.y*1000.0) )
		predicted_s_force = spring_force
		var grip: float = spring_force*TR_friction
		OUTPUT_grip = grip
		var curved_grip_y: float = grip*(TR_MDL_peak_y*(1.0-TR_MDL_linear) +TR_MDL_linear)
		var peaked_grip_y: float = (grip*TR_MDL_peak_y)/TR_MDL_peak_y
		var curved_grip_x: float = grip*(TR_MDL_peak_x*(1.0-TR_MDL_linear) +TR_MDL_linear)
		var peaked_grip_x: float = (grip*TR_MDL_peak_x)/TR_MDL_peak_x
		
		# WHEEL
		var standstill: float = 1.0/(abs(spin/60.0) +1)
		var w_dist: float = (spin - patch_velocity.z/WL_size)
		var predict_dist_x: float = patch_velocity.x
		var predict_slip: float

		STATE_brake_locked = dt_braking>(grip*((TR_spin_resistence_rate*delta)*(dt_overdrive +1.0)))
#		STATE_brake_locked = abs(dt_braking/(dt_overdrive +1.0) -w_dist*0)>grip*(TR_spin_resistence_rate)
		var predicted_grip: float = 0
		if grip>0:
#			predict_slip = max(sqrt(pow(abs(w_dist),2.0) + pow(abs(predict_dist_x),2.0))/(grip*((TR_spin_resistence_rate*delta)/(dt_overdrive +1.0))) -1.0,0)
			predict_slip = max(sqrt(pow(abs(w_dist),2.0) + pow(abs(predict_dist_x),2.0))/(curved_grip_y*((TR_spin_resistence_rate*delta)/(dt_overdrive +1.0))) -1.0,0)
#			var predict_peaked: float = 1.0 -(1.0/(max(sqrt(pow(abs(w_dist),2.0) + pow(abs(predict_dist_x),2.0))/peaked_grip -1,0) +1.0))
			var predict_peaked: float = 1.0 -(1.0/(max(Vector2(w_dist,predict_dist_x).length()/peaked_grip_y/delta -1,0) +1.0))
#			TR_MDL_linear = 1
			if not STATE_brake_locked:
#				spin -= w_dist/(predict_slip*(predict_peaked*TR_MDL_peak*(1.0-TR_MDL_linear) + TR_MDL_linear) +1)

				var p: float = predict_slip*((predict_peaked/TR_MDL_shape_y)*TR_MDL_peak_y*(1.0-TR_MDL_linear) + TR_MDL_linear) +1
#				var p: float = predict_slip +1

				spin -= w_dist/p
				MEASURE_aligning -= w_dist/p
		
		var soften: float = abs(spin/20.0)
		
		t_stiff_y /= soften*TR_deform_factor +1
		t_damp_y /= soften*TR_deform_factor +1
		t_stiff_x /= soften*TR_deform_factor +1
		t_damp_x /= soften*TR_deform_factor +1

		
		# GROUND
		var b_force_y: float = (dt_braking/(t_stiff_y*delta))/(TR_spin_resistence_rate)
		var t_force_y: float = (dt_torque/(t_stiff_y*delta))/(TR_spin_resistence_rate)
		var b_force_x: float = (dt_braking/(t_stiff_x*delta))/(TR_spin_resistence_rate)
		var t_force_x: float = (dt_torque/(t_stiff_x*delta))/(TR_spin_resistence_rate)

		# y
		patch_pos_y.x -= patch_global_velocity.x*delta
		patch_pos_y.y -= patch_global_velocity.z*delta
		# x
		patch_pos_x.x -= patch_global_velocity.x*delta
		patch_pos_x.y -= patch_global_velocity.z*delta

		var x_patch_dist: Vector2 = Vector2(patch_pos_x.x*c_axis[0].x + patch_pos_x.y*c_axis[0].z,patch_pos_x.x*c_axis[2].x + patch_pos_x.y*c_axis[2].z)
		var y_patch_dist: Vector2 = Vector2(patch_pos_y.x*c_axis[0].x + patch_pos_y.y*c_axis[0].z,patch_pos_y.x*c_axis[2].x + patch_pos_y.y*c_axis[2].z)

		patch_pos_x -= Vector2(c_axis[0].x,c_axis[0].z)*x_patch_dist.x*(1.0 -standstill)
		patch_pos_y -= Vector2(c_axis[0].x,c_axis[0].z)*y_patch_dist.x*(1.0 -standstill)

		var patch_slip_y: float
		var patch_peaked_y: float
		var patch_slip_x: float
		var patch_peaked_x: float

		if not STATE_brake_locked:
			var x_patch_clamp_y_amount: float = b_force_y
			var x_patch_clamp_y: float = max(abs(x_patch_dist.y) -x_patch_clamp_y_amount,0)
			var y_patch_clamp_y_amount: float = b_force_x
			var y_patch_clamp_y: float = max(abs(y_patch_dist.y) -y_patch_clamp_y_amount,0)

			if x_patch_dist.y<0:
				patch_pos_x += Vector2(c_axis[2].x,c_axis[2].z)*x_patch_clamp_y
			else:
				patch_pos_x -= Vector2(c_axis[2].x,c_axis[2].z)*x_patch_clamp_y

			if y_patch_dist.y<0:
				patch_pos_y += Vector2(c_axis[2].x,c_axis[2].z)*y_patch_clamp_y
			else:
				patch_pos_y -= Vector2(c_axis[2].x,c_axis[2].z)*y_patch_clamp_y
		
		if grip>0:
			patch_slip_y = max(patch_pos_y.length()/(curved_grip_y/t_stiff_y) -1.0,0)
			patch_peaked_y = 1.0 -(1.0/(max(patch_pos_y.length()/(peaked_grip_y/t_stiff_y) -1,0) +1.0))
			patch_slip_x = max(patch_pos_x.length()/(curved_grip_x/t_stiff_x) -1.0,0)
			patch_peaked_x = 1.0 -(1.0/(max(patch_pos_x.length()/(peaked_grip_x/t_stiff_x) -1,0) +1.0))
		
		patch_pos_y /= patch_slip_y*((patch_peaked_y/TR_MDL_shape_y)*TR_MDL_peak_y*(1.0-TR_MDL_linear) + TR_MDL_linear) +1
		patch_pos_x /= patch_slip_x*((patch_peaked_x/TR_MDL_shape_x)*TR_MDL_peak_x*(1.0-TR_MDL_linear) + TR_MDL_linear) +1

		if not STATE_brake_locked:
			patch_pos_y += Vector2(t_force_y*c_axis[2].x,t_force_y*c_axis[2].z)
			patch_pos_x += Vector2(t_force_x*c_axis[2].x,t_force_x*c_axis[2].z)
		
		var patch_dist_global_x: Vector2
		patch_dist_global_x.x = patch_pos_x.x*c_axis[0].x + patch_pos_x.y*c_axis[0].z
		patch_dist_global_x.y = patch_pos_x.x*c_axis[2].x + patch_pos_x.y*c_axis[2].z
		var patch_dist_global_y: Vector2
		patch_dist_global_y.x = patch_pos_y.x*c_axis[0].x + patch_pos_y.y*c_axis[0].z
		patch_dist_global_y.y = patch_pos_y.x*c_axis[2].x + patch_pos_y.y*c_axis[2].z

		var dist_y: Vector2 = Vector2(patch_velocity.x*t_damp_y -patch_dist_global_y.x*t_stiff_y,(patch_velocity.z - spin*WL_size)*t_damp_y -patch_dist_global_y.y*t_stiff_y)
		var dist_x: Vector2 = Vector2(patch_velocity.x*t_damp_x -patch_dist_global_x.x*t_stiff_x,(patch_velocity.z - spin*WL_size)*t_damp_x -patch_dist_global_x.y*t_stiff_x)
		if grip>0:
			if STATE_brake_locked:
				spin -= w_dist/(predict_slip +1)
				MEASURE_aligning -= w_dist/(predict_slip +1)
#			else:
#				dist_y.y -= (dt_torque/hz_scale)/(TR_spin_resistence_rate/60.0)
#				dist_x.y -= (dt_torque/hz_scale)/(TR_spin_resistence_rate/60.0)
#				dist.y += ((dt_braking/hz_scale)*clamp(patch_velocity.z*t_damp,-1,1))/(TR_spin_resistence_rate/60.0)

		var slip_y: float
		var peaked_y: float
		var slip_x: float
		var peaked_x: float
		if grip>0:
			slip_y = max(dist_y.length()/curved_grip_y -1.0,0)
			peaked_y = 1.0 -(1.0/(max(dist_y.length()/peaked_grip_y -1,0) +1.0))
			slip_x = max(dist_x.length()/curved_grip_x -1.0,0)
			peaked_x = 1.0 -(1.0/(max(dist_x.length()/peaked_grip_x -1,0) +1.0))
			
			OUTPUT_skidding = min(slip_y*curved_grip_y,slip_x*curved_grip_x)
			OUTPUT_stressing = max(dist_y.length(),dist_x.length())*min(abs(spin),1)
			if name == "fl":
				_debug.queue["OUTPUT_skidding"] = OUTPUT_skidding
				_debug.queue["OUTPUT_stressing"] = OUTPUT_stressing

		dist_y /= slip_y*((peaked_y/TR_MDL_shape_y)*TR_MDL_peak_y*(1.0-TR_MDL_linear) + TR_MDL_linear) +1
		dist_x /= slip_x*((peaked_x/TR_MDL_shape_x)*TR_MDL_peak_x*(1.0-TR_MDL_linear) + TR_MDL_linear) +1

		if grip>0:
			$c_v/lateral.scale.y = dist_x.x/40.0
			if $c_v/lateral.scale.y>0:
				$c_v/lateral.rotation_degrees.x = 91
			else:
				$c_v/lateral.rotation_degrees.x = 89
			if slip_x>0:
				$c_v/lateral.modulate = Color(1,0,0)
			else:
				var debug_effective: float = 1.0 +((dist_x.length() -curved_grip_x)/(curved_grip_x))
				var green: float = debug_effective
				var blue: float = 1.0 - debug_effective
				$c_v/lateral.modulate = Color(0,green,blue)
			
			$c_v/longitudinal.scale.y = dist_y.y/40.0
			if $c_v/longitudinal.scale.y>0:
				$c_v/longitudinal.rotation_degrees.x = 91
			else:
				$c_v/longitudinal.rotation_degrees.x = 89
			if slip_y>0:
				$c_v/longitudinal.modulate = Color(1,0,0)
			else:
				var debug_effective: float = 1.0 +((dist_y.length() -curved_grip_y)/(curved_grip_y))
				var green: float = debug_effective
				var blue: float = 1.0 - debug_effective
				$c_v/longitudinal.modulate = Color(0,green,blue)

		$c_v/suspforce.scale.y = spring_force/40.0

#		dist = dist.limit_length(grip)
		
		var forces: Vector3
		forces += c_axis.y*spring_force
		if grip>0:
			forces -= c_axis.x*dist_x.x
			forces -= c_axis.z*dist_y.y
		
#		var point: Vector3 = c_point-car.global_translation
		var point: Vector3 = car.PSDB.center_of_mass
		
		point -= (point - (c_point-car.global_translation))/car.PHYS_form_factor
		
		$imp_point.global_translation = point +car.global_translation
		
		impulse = [point,forces*hz_scale]
	else:
		OUTPUT_compressed = 0
		OUTPUT_skidding = 0
		OUTPUT_grip = 0
		OUTPUT_stressing = 0
		TYRE_deflated = 0
		TYRE_vertical_v = 0
		prev_compressed = 0.0
		c_v.global_translation = c_point
		impulse = [Vector3(),Vector3()]
		patch_pos_y *= 0
		patch_pos_x *= 0
		
	p_py.translation.x = patch_pos_y.x*c_axis[0].x + patch_pos_y.y*c_axis[0].z
	p_py.translation.z = patch_pos_y.x*c_axis[2].x + patch_pos_y.y*c_axis[2].z
	p_px.translation.x = patch_pos_x.x*c_axis[0].x + patch_pos_x.y*c_axis[0].z
	p_px.translation.z = patch_pos_x.x*c_axis[2].x + patch_pos_x.y*c_axis[2].z
	
	# vv 2.0
	past_global_axle_pos = global_translation
	# end
	
	AN_hub.global_translation = c_point+global_transform.basis.orthonormalized().xform(Vector3(0,real_wheelsize - TYRE_deflated,0))
	if not STATE_brake_locked:
		AN_spin.rotate_x(spin*delta*2.0)
