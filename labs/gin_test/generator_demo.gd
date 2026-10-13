#tool
extends AudioStreamPlayer

export var test_influence: float = 1.0
export var test_loader: bool
export var test_maxvol: float = 1.0

export var DB_draw_waveforms: bool
export var sample: AudioStreamSample setget set_sample
export var sample_arr: PoolRealArray
func set_sample(val):
	sample = val
	var newarr = read_8bit_samples(sample)
	sample_arr = newarr
	property_list_changed_notify()

var playback: AudioStreamPlayback = get_stream_playback()
export var sample_hz: int = 44100 # Keep the number of samples to mix low, GDScript is not super fast.

export var lookup_offset: int setget xlookup_offset

func xlookup_offset(val):
	lookup_offset = val
	findloop(look_sample)

export var lookup_size: int = 240
export var lookup_limit: int = 100
export var look_sample: int = 100724 setget findloop
func findloop(val):
	return
	if not selected_keyframe == -1:
		return
#	$reference.clear_points()
	next_similar_waves.clear()
	previous_similar_waves.clear()
	var from: int = val
	var to: int = val + lookup_size
	
	
	
	if DB_draw_waveforms:
		for i in range(lookup_size):
			var _wave: float = sample_arr[from + i]
			previous_similar_waves.append(_wave)
			$reference.add_point(Vector2(i,_wave*240),i)
	else:
		for i in range(lookup_size):
			var _wave: float = sample_arr[from + i]
			previous_similar_waves.append(_wave)
	
	var leaderboard: Dictionary
	var leaderboard_ids: Array
	var stepped: int
	
	to += lookup_offset
	
	while true:
		var _buffer: Array
		var _score: float
		
		for i in range(lookup_size):
			var _wave: float = sample_arr[to + i]
			var _dist: float = _wave - previous_similar_waves[i]
			_buffer.append(_wave)
			_score += abs(_dist)
		
		leaderboard[_score] = _buffer
		leaderboard_ids.append(to)
		to += 1
		stepped += 1
		if to>sample_arr.size() or stepped>lookup_limit:
#			printerr("lookup timeout")
			break
	
	var lowest: float = 99999
	var end_position: int
	var l_id: int
	for i in leaderboard:
		if lowest>i:
			lowest = i
			end_position = leaderboard_ids[l_id]
		l_id += 1
	
	next_similar_waves = leaderboard[lowest]
	
	look_sample = val
	
	clip_start = val
	clip_end = end_position
	
	$end.clear_points()
	$start.clear_points()
	$stitched.clear_points()
	
	previous_similar_pos = val
	next_similar_pos = end_position
	calibrate_clip_length = end_position - val
	
	# ok
	
	lookup_limit = calibrate_clip_length
#	lookup_offset = calibrate_clip_length - calibrate_clip_offset
	
	# ok end
	
	if DB_draw_waveforms:
		var skipprv: bool
		for i in range(previous_similar_waves.size()):
			if skipprv:
				skipprv = false
			else:
				skipprv = true
				var _wave: float = previous_similar_waves[i]
				$start.add_point(Vector2(i,_wave*240),i)
		skipprv = false
		for i in range(next_similar_waves.size()):
			if skipprv:
				skipprv = false
			else:
				skipprv = true
				var _wave: float = next_similar_waves[i]
				$end.add_point(Vector2(i,_wave*240),i)
	
	
		var created: int
		
		
		var st_distance: int = end_position - val
		var skip: bool
		print(st_distance)
		$stitched.clear_points()
		for i in range(st_distance):
			if skip:
				skip = false
			else:
				skip = true
				var _wave: float = sample_arr[from + i]
				previous_similar_waves.append(_wave)
				$stitched.add_point(Vector2(i,_wave*240),i)

#	print([val,end_position])

var iterated_float: float

var next_similar_waves: Array
var previous_similar_waves: Array

var next_similar_pos: int
var previous_similar_pos: int
export var calibrate_clip_length: int
export var calibrate_clip_offset: int

export var latency_reduction: int = 2500
export var speed: float = 1
export var clip_start: int setget xclip_start
func xclip_start(val):
	clip_start = val
	draw_clip()
	
export var clip_end: int setget xclip_end
func xclip_end(val):
	clip_end = val
	draw_clip()
export var clip_find_zero_crossings: bool setget xclip_find_zero_crossings
func xclip_find_zero_crossings(val):
	if !val:
		return
	
	var current_start_sample: float = sample_arr[clip_start]
	var current_end_sample: float = sample_arr[clip_end]
	
	var start_is_up: bool = current_start_sample>0
	var end_is_up: bool = current_end_sample>0
	
	while true:
		if sample_arr[clip_start]<0 and start_is_up or sample_arr[clip_start]>0 and !start_is_up:
			break
		clip_start += 1
		clip_end += 1

	draw_clip()

export var lookup_size_regions: Curve
export var lookup_offset_regions: Curve

export var render_up: bool
export var render_keyframes: bool

export var find_similar_next: bool setget xfind_similar_next
export var set_lookup_values_to_curve: bool setget xset_lookup_values_to_curve
func xset_lookup_values_to_curve(val, refresh: bool = true):
	lookup_size = lookup_size_regions.interpolate_baked(float(look_sample)/float(sample_arr.size()))
	lookup_offset = lookup_offset_regions.interpolate_baked(float(look_sample)/float(sample_arr.size()))
	if refresh:
		findloop(look_sample)
		property_list_changed_notify()

func xfind_similar_next(val):
	if !val:
		return
	xset_lookup_values_to_curve(true, false)
	findloop(next_similar_pos)

export var keyframes: Array
export var selected_keyframe: int = -1 setget xselected_keyframe
export var draw_clip_on_keyframe_switch: bool
export var refresh_props_on_keyframe_switch: bool
func xselected_keyframe(val):
	selected_keyframe = clamp(val, -1, keyframes.size() -1)
	clip_start = keyframes[selected_keyframe][0]
	clip_end = keyframes[selected_keyframe][1]
	if draw_clip_on_keyframe_switch:
		draw_clip()
	if refresh_props_on_keyframe_switch:
		property_list_changed_notify()
	
export var correct_keyframe: bool setget xcorrect_keyframe
func xcorrect_keyframe(val):
	if !val:
		return
	keyframes[selected_keyframe][0] = clip_start
	keyframes[selected_keyframe][1] = clip_end
	
export var keyframe_align_to_zero: bool setget xkeyframe_align_to_zero
func xkeyframe_align_to_zero(val):
	if !val:
		return
	
	var start_is_up: bool = sample_arr[clip_start]>0
	var end_is_up: bool = sample_arr[clip_end]>0
	
	while true:
		if !start_is_up and sample_arr[clip_start]>0 or start_is_up and !sample_arr[clip_start]>0:
			break
		clip_start += 1
		clip_end += 1
	
	draw_clip()
	property_list_changed_notify()

export var align_all_keyframes_to_zero: bool setget xalign_all_keyframes_to_zero
func xalign_all_keyframes_to_zero(val):
	if !val:
		return
	while true:
		xkeyframe_align_to_zero(true)
		xcorrect_keyframe(true)
		xselected_keyframe(selected_keyframe +1)
		if selected_keyframe == keyframes.size()-1:
			break

export var keyframe_move_direction: int
export var keyframe_adapt_pitch: bool
export var keyframe_target_pitch: float = 1.0

export var baked_rev_ids: Curve
export var bake_keyframes: bool setget xbake_keyframes
func xbake_keyframes(val):
	if !val:
		return
	baked_rev_ids = Curve.new()
	baked_rev_ids.max_value = keyframes.size()-1
	
	var shortest: int = (keyframes[keyframes.size()-1][1] - keyframes[keyframes.size()-1][0])
	
	var highest: int = -1
	
	for i in range(keyframes.size()):
		var pitch: float = float(i)/float(keyframes.size()-1)
		var found_length: float
		var found_id: int
		var dist: float = 999999999.0
		if pitch>0:
			var expectation: float = float(shortest)/pitch
			for o in range(keyframes.size()):
				var leg: float = keyframes[o][1] - keyframes[o][0]
				var dist2: float = abs(leg - expectation)
				if dist>dist2:
					dist = dist2
					found_length = leg
					found_id = o
				
			print("expecting %f, found %f, id is %d" % [expectation, found_length, found_id])
		if found_id>highest:
			highest = found_id
			baked_rev_ids.add_point(Vector2(pitch, found_id), 0, 0, Curve.TANGENT_LINEAR, Curve.TANGENT_LINEAR)
		else:
			printerr("index went down")
	
	property_list_changed_notify()
export var baked_rev_ids_into_array: bool setget xbaked_rev_ids_into_array
func xbaked_rev_ids_into_array(val):
	if !val:
		return
	
	var newarr: Array
	
	for i in range(keyframes.size()):
		var progress: float = i/float(keyframes.size() -1)
		newarr.append(int(baked_rev_ids.interpolate(progress)))
	
	arrayed_rev_ids = newarr
	property_list_changed_notify()
	
export var arrayed_rev_ids: Array

export var bake_revtest: float setget xbake_revtest
export var bake_revtest_interpolation_weight: float = 10.0
func xbake_revtest(val):
#	xselected_keyframe(int(baked_rev_ids.interpolate_baked(val)))
	bake_revtest = val
#	$stitched.clear_points()
	
#	var skip: bool
#	var from: int = clip_start
#	for i in range(keyframes[selected_keyframe][1] - keyframes[selected_keyframe][0]):
#		if skip:
#			skip = false
#		else:
#			skip = true
#			var _wave: float = sample_arr[from + i]
#			previous_similar_waves.append(_wave)
#			$stitched.add_point(Vector2(i,_wave*240),i)
var bake_revtest_interp: float

export var bake_adapt_pitch: bool
export var baked_running: bool
export(int, 0, 2) var baked_runtime_method: int
export var baked_reverse: bool
export var baked_chop_prevention: float = 100.0
var wave_offset: float

export var EDITOR_refresh_props: bool setget xEDITOR_refresh_props
func xEDITOR_refresh_props(val):
	if !val:
		return
	
	draw_clip()
			
		
	property_list_changed_notify()

var allowed_start: int
var allowed_end: int

func get_relative_length_of_keyframe(idx: int) -> int:
	return (keyframes[idx][1] - keyframes[idx][0])/(keyframes[keyframes.size()-1][1] - keyframes[keyframes.size()-1][0])

func draw_clip():
	var skip: int
	var from: int = clip_start
#	$stitched.clear_points()
#	for i in range(clip_end - clip_start):
#		if skip == 0 or i == (clip_end - clip_start)-1:
#			var _wave: float = sample_arr[from + i]
#			previous_similar_waves.append(_wave)
#			$stitched.add_point(Vector2(i,_wave*240),i)
#		skip += 1
#		if skip>2:
#			skip = 0

var current_id: int
var next_id: int
var progress: float
var allowed_start2: int
var allowed_end2: int
var length_mod: float

var chop_preventor_idpos: float

var last_buffer: float

var print_skip: int

func _fill_buffer(pb: AudioStreamPlayback):
	var to_fill: int = pb.get_frames_available()
	var clipping: bool
	
	var kf_size: float = float(keyframes.size())
	var max_id: int = keyframes.size() -1
	var max_idf: float = float(keyframes.size() -1) 
	
	if clip_end>0 or clip_start>0:
		clipping = true

	var paren: SVD_BODY = get_parent()

	if test_loader:
		volume_db = linear2db(paren.throttle*test_maxvol)
	else:
		volume_db = linear2db((1.0 -paren.throttle)*test_maxvol)


	if baked_running:
		if baked_runtime_method == 0:
			while to_fill > latency_reduction:
				print_skip += 1
				if print_skip>=100:
					print_skip = 0
				bake_revtest_interp = lerp(bake_revtest_interp, bake_revtest, bake_revtest_interpolation_weight/float(latency_reduction))

				speed = bake_revtest_interp*float(max(allowed_end, 128) - allowed_start)/float(keyframes[max_id][1] - keyframes[max_id][0])
				var iterated_float_next: float
				var buffer: float
				if baked_reverse:
					iterated_float -= speed
					if iterated_float<=allowed_start:
						current_id = arrayed_rev_ids[int(min(bake_revtest_interp*kf_size, max_idf))]
						
						var past_sample: float = sample_arr[int(iterated_float)]

						allowed_start = keyframes[current_id][0]
						allowed_end = keyframes[current_id][1]
						
						iterated_float = keyframes[current_id][1]
						buffer = sample_arr[int(iterated_float)] + wave_offset
						wave_offset += last_buffer - buffer
				else:
					iterated_float += speed
					if iterated_float>=allowed_end:
						current_id = arrayed_rev_ids[int(min(bake_revtest_interp*kf_size, max_idf))]
						
						var past_sample: float = sample_arr[int(iterated_float)]

						allowed_start = keyframes[current_id][0]
						allowed_end = keyframes[current_id][1]
						
						iterated_float = keyframes[current_id][0]

						buffer = sample_arr[int(iterated_float)] + wave_offset
						wave_offset += last_buffer - buffer

				buffer = sample_arr[int(iterated_float)] + wave_offset
				wave_offset = lerp(wave_offset, 0, 1.0/max(baked_chop_prevention, 1.0))

				pb.push_frame(Vector2.ONE*buffer)
				last_buffer = buffer
	#			pb.push_frame(Vector2.ONE*sample_arr[int(iterated_float)])
				to_fill -= 1
		elif baked_runtime_method == 1:
			while to_fill > latency_reduction:
				bake_revtest = paren.rpm*test_influence/8500.0
				print_skip += 1
				if print_skip>=100:
					print_skip = 0
				bake_revtest_interp = lerp(bake_revtest_interp, bake_revtest, bake_revtest_interpolation_weight/float(latency_reduction))

				speed = bake_revtest_interp*float(max(allowed_end, 128) - allowed_start)/float(keyframes[max_id][1] - keyframes[max_id][0])
				var iterated_float_next: float
				var buffer: float
				if baked_reverse:
					iterated_float -= speed
					if iterated_float<=allowed_start:
						var lookthrough: float = min(bake_revtest_interp*kf_size, max_idf)
						var int_lookthrough: int = int(lookthrough)
						var int_lookthrough_advanced: int = min(int(lookthrough) +1, max_id)
						
						var current_id_float: float = lerp(
							arrayed_rev_ids[int_lookthrough],
							arrayed_rev_ids[int_lookthrough_advanced],
							lookthrough - int_lookthrough)
						
						current_id = int(current_id_float)
						next_id = min(current_id + 1, max_id)
						progress = 1.0 -(next_id - current_id_float)
						
						var past_sample: float = sample_arr[int(iterated_float)]

						allowed_start = keyframes[current_id][0]
						allowed_end = keyframes[current_id][1]
						
						allowed_start2 = keyframes[next_id][0]
						allowed_end2 = keyframes[next_id][1]
						length_mod = float(keyframes[next_id][1] - keyframes[next_id][0])/float(keyframes[current_id][1] - keyframes[current_id][0])

						iterated_float = keyframes[current_id][1]

						iterated_float_next = allowed_start2 + (iterated_float -keyframes[current_id][0])*length_mod
						
						buffer = lerp(sample_arr[int(iterated_float)], sample_arr[int(iterated_float_next)], progress) + wave_offset
						
						wave_offset += last_buffer - buffer
				else:
					iterated_float += speed
					if iterated_float>=allowed_end:
						var lookthrough: float = min(bake_revtest_interp*kf_size, max_idf)
						var int_lookthrough: int = int(lookthrough)
						var int_lookthrough_advanced: int = min(int(lookthrough) +1, max_id)
						
						var current_id_float: float = lerp(
							arrayed_rev_ids[int_lookthrough],
							arrayed_rev_ids[int_lookthrough_advanced],
							lookthrough - int_lookthrough)
						
						current_id = int(current_id_float)
						next_id = min(current_id + 1, max_id)
						progress = 1.0 -(next_id - current_id_float)
						
						var past_sample: float = sample_arr[int(iterated_float)]

						allowed_start = keyframes[current_id][0]
						allowed_end = keyframes[current_id][1]
						
						allowed_start2 = keyframes[next_id][0]
						allowed_end2 = keyframes[next_id][1]
						length_mod = float(keyframes[next_id][1] - keyframes[next_id][0])/float(keyframes[current_id][1] - keyframes[current_id][0])

						iterated_float = keyframes[current_id][0]

						iterated_float_next = allowed_start2 + (iterated_float -keyframes[current_id][0])*length_mod
						
						buffer = lerp(sample_arr[int(iterated_float)], sample_arr[int(iterated_float_next)], progress) + wave_offset
						
						
						wave_offset += last_buffer - buffer

				iterated_float_next = allowed_start2 + (iterated_float -keyframes[current_id][0])*length_mod
				
				buffer = lerp(sample_arr[int(iterated_float)], sample_arr[int(iterated_float_next)], progress) + wave_offset
				wave_offset = lerp(wave_offset, 0, 1.0/max(baked_chop_prevention, 1.0))

				pb.push_frame(Vector2.ONE*buffer)
				last_buffer = buffer
	#			pb.push_frame(Vector2.ONE*sample_arr[int(iterated_float)])
				to_fill -= 1
		elif baked_runtime_method == 2:
			while to_fill > latency_reduction:
				print_skip += 1
				if print_skip>=100:
					print_skip = 0
				bake_revtest_interp = lerp(bake_revtest_interp, bake_revtest, bake_revtest_interpolation_weight/float(latency_reduction))

				speed = bake_revtest_interp*float(max(allowed_end, 128) - allowed_start)/float(keyframes[max_id][1] - keyframes[max_id][0])
				var iterated_float_next: float
				var buffer: float
				if baked_reverse:
					iterated_float -= speed
					if iterated_float<=allowed_start:
						current_id = arrayed_rev_ids[int(min(bake_revtest_interp*kf_size, max_idf))]
						
						var past_sample: float = sample_arr[int(iterated_float)]

						allowed_start = keyframes[current_id][0]
						allowed_end = keyframes[current_id][1]
						
						iterated_float = keyframes[current_id][1]
						buffer = sample_arr[int(iterated_float)] + wave_offset
						wave_offset += last_buffer - buffer
				else:
					iterated_float += speed
					if iterated_float>=allowed_end:
						var lookthrough: float = min(bake_revtest_interp*kf_size, max_idf)
						var int_lookthrough: int = int(lookthrough)
						var int_lookthrough_advanced: int = min(int(lookthrough) +1, max_id)
						
						var current_id_float: float = lerp(
							arrayed_rev_ids[int_lookthrough],
							arrayed_rev_ids[int_lookthrough_advanced],
							lookthrough - int_lookthrough)
						
						current_id = arrayed_rev_ids[int_lookthrough]
						next_id = arrayed_rev_ids[int_lookthrough_advanced]
						progress = lookthrough - int_lookthrough
						var past_sample: float = sample_arr[int(iterated_float)]
	
						print([current_id, next_id, progress, current_id_float])
#						progress = 1
						allowed_start = lerp(keyframes[current_id][0], keyframes[next_id][0], progress)
						allowed_end = lerp(keyframes[current_id][1], keyframes[next_id][1], progress)
						
						iterated_float = lerp(keyframes[current_id][0], keyframes[next_id][0], progress)

						buffer = sample_arr[int(iterated_float)] + wave_offset
						wave_offset += last_buffer - buffer

				buffer = sample_arr[int(iterated_float)] + wave_offset
				wave_offset = lerp(wave_offset, 0, 1.0/max(baked_chop_prevention, 1.0))

				pb.push_frame(Vector2.ONE*buffer)
				last_buffer = buffer
	#			pb.push_frame(Vector2.ONE*sample_arr[int(iterated_float)])
				to_fill -= 1
	
	else:
		while to_fill > latency_reduction:
			
			bake_revtest_interp = lerp(bake_revtest_interp, bake_revtest, bake_revtest_interpolation_weight/float(latency_reduction))
			
			if keyframe_adapt_pitch:
				speed = keyframe_target_pitch*(allowed_end - allowed_start)/(keyframes[keyframes.size()-1][1] - keyframes[keyframes.size()-1][0])
			elif bake_adapt_pitch:
				speed = bake_revtest_interp*(allowed_end - allowed_start)/(keyframes[keyframes.size()-1][1] - keyframes[keyframes.size()-1][0])

			iterated_float += speed
			
			if clipping:
				if speed>0:
					if iterated_float>=allowed_end:
						iterated_float = clip_start
						allowed_start = clip_start
						allowed_end = clip_end
						if not keyframe_move_direction == 0:
							xselected_keyframe(max(selected_keyframe +keyframe_move_direction, 0))
				else:
					if iterated_float<=allowed_start:
						iterated_float = clip_end
						allowed_start = clip_start
						allowed_end = clip_end
						if not keyframe_move_direction == 0:
							xselected_keyframe(max(selected_keyframe +keyframe_move_direction, 0))
			else:
				if iterated_float>=sample_arr.size():
					iterated_float = speed
				elif iterated_float<=0.0:
					iterated_float = sample_arr.size()-1
			
			pb.push_frame(Vector2.ONE*sample_arr[int(iterated_float)])
			to_fill -= 1


func kickstart(pb: AudioStreamPlayback):
	var to_fill: int = pb.get_frames_available()
	while to_fill > 0:
		pb.push_frame(Vector2.ONE)
		to_fill -= 1

func _ready():
	stream.mix_rate = sample_hz
	playback = get_stream_playback()
	kickstart(playback)
	play()

func _process(delta):
	_fill_buffer(playback)

	if render_up:
		xfind_similar_next(true)
		if render_up:
			keyframes.append([clip_start, clip_end])

static func read_8bit_samples(stream: AudioStreamSample) -> Array:
	var _samples: Array
	assert(stream.format == AudioStreamSample.FORMAT_8_BITS)
	var bytes = stream.data
	for i in len(bytes):
		var b = bytes[i]
		# Despite what the doc says, PoolByteArray contains uint8_t values,
		# which are unsigned bytes representing signed numbers.
		# In GDScript, we still get positive integers, i.e -2 => 253.
		# So we bring back their representation as unsigned,
		# emulating the 8-bit wrapping behavior.
		var u = (b + 128) & 0xff
		# Then bring back to signed -1..1 range
		var s = float(u - 128) / 128.0
		_samples.append(s)
	return _samples
