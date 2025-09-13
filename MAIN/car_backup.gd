extends RigidBody
#class_name SVD_BODY

var engine_resistance: float
var runtime_engine_resistance: float
var rpm: float
var past_rpm: float

var wheels: Array
var gear: int
var past_speed: float
var past_rpmspeed: float
var rt_dsweight: float

export var EN_DriveForce: float = 3
export(int, "Wheels Synced", "Wheels Desynced") var EN_DriveForceBehavior
export var EN_RevUpSpeed: float = 150
export var EN_RevDownSpeed: float = 50
export var EN_RevAlignmentRate: float = 50
export var EN_IdleRPM: float = 800
export var EN_MaxRPM: float = 7000
export var EN_CanStall: bool = false
export var EN_CanOverRev: bool = false
export var EN_StallPrevention: float = 2

export(Array, float) var GB_ForwardGearRatios = [
	3.321,
	1.902,
	1.308,
	1.0,
	0.759,
]
export var GB_FinalDriveRatio: float = 4.083
export(Array, float) var GB_ReverseGearRatios = [
	3.0,
]

var throttle: float
var IP_throttle: float
var dt_dropping: float

const rads2rpm: float = 9.549297

func refresh_wheels():
	wheels = []
	for i in get_children():
		if i is SVD_WHEEL:
			wheels.append(i)

func _ready():
	refresh_wheels()
	
func _input(event):
	if event.is_action_pressed("shiftup") and gear<len(GB_ForwardGearRatios):
		gear += 1
	elif event.is_action_pressed("shiftdown") and gear>-(len(GB_ReverseGearRatios)):
		gear -= 1

func gearbox(dt) -> float:
	var rat: float = 1
	
	if gear>0:
		rat = GB_ForwardGearRatios[gear-1]
	elif gear<0:
		rat = GB_ReverseGearRatios[abs(gear)-1]
	
	return rat*GB_FinalDriveRatio
	
func engine(dt) -> Array:
	IP_throttle = Input.get_action_strength("accelerate")
	
	throttle -= (throttle - IP_throttle)*0.5
	
	var midpoint: float = EN_RevDownSpeed/(EN_RevUpSpeed +EN_RevDownSpeed)
	var from_idle: float = 1.0 -rpm/EN_IdleRPM
	
	if rpm<EN_IdleRPM+EN_RevDownSpeed:
		if EN_CanStall:
			throttle = max(throttle,min(midpoint*min(EN_StallPrevention,1) +from_idle*max(EN_StallPrevention-1,0),1))
		else:
			rpm = EN_IdleRPM +EN_RevDownSpeed
	elif rpm>EN_MaxRPM-EN_RevUpSpeed:
		if EN_CanOverRev:
			throttle = 0.0
		else:
			rpm = EN_MaxRPM -EN_RevUpSpeed
	var torque_measure: float = throttle*EN_RevUpSpeed - EN_RevDownSpeed*(1.0 -throttle)
	rpm += torque_measure
	if not EN_CanStall and rpm<=EN_IdleRPM:
		torque_measure *= 0
#	print(torque_measure)
	
	return [torque_measure/rads2rpm,max(EN_RevUpSpeed,EN_RevDownSpeed)/rads2rpm,torque_measure]


func _physics_process(delta):
	var ds_weight: float
	var rpm_speed: float
	
	var g_ratio: float = gearbox(delta)*2.0
	
	if Input.is_action_pressed("accelerate"):
		throttle = 1
	else:
		throttle = 0
	
	var test_maxtorque: float = 10.0/g_ratio
	
	var test_drive_torque: float = 0.2
	test_drive_torque *= g_ratio
	
	var test_overdrive: float = max(test_drive_torque/test_maxtorque -1.0,0)
	var test_cgrip: float = max(EN_RevUpSpeed,EN_RevDownSpeed) + EN_RevAlignmentRate ; if Input.is_action_pressed("clutch") or gear == 0:
										test_cgrip = 0
	var test_cstab: float = 1
	var test_cstab_rads: float = test_cstab*rads2rpm
	var test_cgrip_rads: float = test_cgrip/rads2rpm
	var test_dsweight: float = EN_RevUpSpeed/EN_DriveForce
#	var test_dt: float = 5
#	test_overdrive *= 0
	
#	print([gear,test_maxtorque,test_drive_torque,test_overdrive])

#	print("kph: %d" % (linear_velocity.length()*3.6))
	print("gs: %f" % ((linear_velocity.length() - past_speed)*9.8))
	past_speed = linear_velocity.length()
	var cs_rads: float = rpm/rads2rpm
	var torque_data: Array = engine(delta)
	
	rt_dsweight = max(rt_dsweight,1)
	
	
	
	# steering
	var steer: float = (get_viewport().get_mouse_position().x/OS.get_real_window_size().x -0.5)*-2.0

	if steer>0:
		steer = pow(abs(steer),1.1)
	else:
		steer = -pow(abs(steer),1.1)

	var circle_point: float = -1.151
	var turnradius: float = 4.0

#	steer = 0
	
	if abs(steer)>0:
		turnradius /= abs(steer)
		if steer<0:
			turnradius *= -1.0
		$fl.rotation_degrees.y = rad2deg(atan((-circle_point + $fl.translation.z)/(turnradius - $fl.translation.x)))
		$fr.rotation_degrees.y = rad2deg(atan((-circle_point + $fr.translation.z)/(turnradius - $fr.translation.x)))
	else:
		$fl.rotation.y = 0
		$fr.rotation.y = 0

	$"../sw".rect_rotation = -steer*420
	$"../sw_desired".rect_rotation = -steer*420

	
	var DB_SLIP: float
	for wheel in wheels:
		wheel = wheel as SVD_WHEEL # cast placeholder
		
		ds_weight += wheel.DT_influence
		rpm_speed += wheel.spin*wheel.DT_influence
		
		var dt_dist: float = ((cs_rads/g_ratio - wheel.spin)/test_cstab)*rads2rpm
		if gear<0:
			dt_dist = ((cs_rads/g_ratio + wheel.spin)/test_cstab)*rads2rpm
		if not EN_CanStall and rpm<=EN_IdleRPM:
			dt_dist *= 0
		
		if EN_DriveForceBehavior == 0 and abs(dt_dist)>test_cgrip_rads or EN_DriveForceBehavior == 1 and not dt_dropping == 0:
			if EN_DriveForceBehavior:
				dt_dist = dt_dropping
			DB_SLIP = 1
			wheel.dt_overdrive = 1
			
			var ok: float = test_cgrip_rads
			if dt_dist<0:
				ok = -test_cgrip_rads
				
			if gear<0:
				wheel.dt_torque = -(ok*wheel.DT_influence*g_ratio/rt_dsweight)/test_dsweight
			else:
				wheel.dt_torque = (ok*wheel.DT_influence*g_ratio/rt_dsweight)/test_dsweight
		else:
			var w_torque: float = (torque_data[0]*wheel.DT_influence*g_ratio/rt_dsweight)/test_dsweight
			var w_torque_est: float = (torque_data[1]*wheel.DT_influence*g_ratio/rt_dsweight)/test_dsweight
			w_torque = clamp(w_torque,-test_cgrip_rads,test_cgrip_rads)
			w_torque_est = clamp(w_torque_est,-test_cgrip_rads,test_cgrip_rads)
			var w_overdrive: float = max(w_torque_est/(torque_data[1]/g_ratio) -1.0,0)
			wheel.dt_overdrive = w_overdrive
			if gear<0:
				wheel.dt_torque = -w_torque
			else:
				wheel.dt_torque = w_torque
			
		
		if Input.is_action_pressed("brake"):
			wheel.dt_braking = wheel.DT_BrakeTorque
		else:
			wheel.dt_braking = 0
	
	$DB_clutchslip.unit_db = max(linear2db(DB_SLIP*0.5),-80)
	rt_dsweight = ds_weight
	rpm_speed *= (rads2rpm/ds_weight)*g_ratio
	if gear<0:
		rpm_speed = -rpm_speed
	
	var est_rpm_dist: float = rpm - rpm_speed
	rpm -= clamp(est_rpm_dist,-test_cgrip,test_cgrip)
	if est_rpm_dist>test_cgrip:
		dt_dropping = 1
	elif est_rpm_dist<-test_cgrip:
		dt_dropping = -1
	else:
		dt_dropping = 0
	rpm += clamp(est_rpm_dist,-test_cgrip,test_cgrip)
	
	if not EN_CanOverRev and rpm_speed>EN_MaxRPM:
		rpm_speed = EN_MaxRPM
	elif not EN_CanStall and rpm_speed<EN_IdleRPM:
		rpm_speed = EN_IdleRPM
		
	var rpm_dist: float = rpm - rpm_speed
	rpm -= clamp(rpm_dist,-test_cgrip,test_cgrip)
	
#	print(rpm - past_rpm)
#	print(cs_rads)
	past_rpm = rpm
	$enon.unit_db = max(linear2db(throttle),-80.0)
	$enoff.unit_db = max(linear2db(1.0 -throttle),-80.0)
	$enon.max_db = $enon.unit_db
	$enoff.max_db = $enoff.unit_db
	
	$enon.pitch_scale = max(rpm,800)/8000.0
	$enoff.pitch_scale = max(rpm,800)/8000.0
#	print(rpm_speed)
