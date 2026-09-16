extends Node2D

@export var particle_size: float = 4
@export var num_particles: int = 100
@export var gravity: Vector2 = Vector2(0, 980)
@export_range(0.0, 1.0) var wall_damping: float = 0.5

@export var smoothing_scale:float = 16
@export var rest_density:float = 0.25
@export var spacing:float = 8
@export var stiffness:float = 200.0
@export var viscosity: float = 50.0
var mass: float = 0.0

var poly6_const:float = 0.0
var spiky_grad_const:float = 0.0
var visc_lap_const:float = 0.0
var h_sq: float = 0.0

# The starting pos params for the particles.
var xmin: int = -100
var ymin: int = -100
var xmax: int = 100
var ymax: int = 100


# Container postion and size
var container_pos: Vector2 = Vector2(0, 0)
var container_size: Vector2 = Vector2(300, 300)

# Simulation state. One entry per particle — index i is the same
# particle in every array. More arrays get added as you go:
# forces, densities, pressures.
var positions: PackedVector2Array = PackedVector2Array()
var velocities: PackedVector2Array = PackedVector2Array()
var forces: PackedVector2Array = PackedVector2Array()
var densities: PackedFloat32Array = PackedFloat32Array()
var pressures: PackedFloat32Array = PackedFloat32Array()

# Debugging
@export var debug_density: bool = false

# Cached container bounds
var container_bounds: Rect2

func _ready():
	poly6_const = 4.0/(PI * pow(smoothing_scale, 8.0))
	h_sq = smoothing_scale * smoothing_scale
	mass = rest_density * (spacing * spacing)
	spiky_grad_const = -30.0 / (PI * pow(smoothing_scale, 5.0))
	visc_lap_const = 40.0 / (PI * pow(smoothing_scale, 5.0))
	# Cache bounds so we don't recreate the Rect2 every particle
	container_bounds = Rect2(
		container_pos - container_size / 2.0,
		container_size
	)
	for i in range(num_particles):
		spawn_particle()

func spawn_particle():
	# Give each particle a slightly random starting point
	positions.append(Vector2(randf_range(xmin, xmax), randf_range(ymin, ymax)))
	# Starts at rest. Nothing moves it yet — that arrives with the integrator.
	velocities.append(Vector2.ZERO)
	densities.append(rest_density)
	pressures.append(0.0)
	forces.append(Vector2.ZERO)

func _draw():
	draw_rect(container_bounds, Color.WHITE, false, 2.0)
	for pos in range(positions.size()):
		var c := clampf(densities[pos] / rest_density,0.0,1.0)
		var color = Color.BLUE.lerp(Color.RED, c)
		draw_circle(positions[pos], particle_size, color)

# The box the fluid lives in. Used for drawing now, and for
# boundary resolution once particles start moving.
func bounds() -> Rect2:
	return Rect2(container_pos - container_size / 2, container_size)
	
func resolve_bounds(i: int):
#	Get all the varibles to peform the calculations
	var b = bounds()
	var r = particle_size
	var pos = positions[i]
	var vel = velocities[i]
	
#	Check if the particle is outside the bounds and if it is apply wall dampening and send in opposite direction
# 	Left
	if pos.x < b.position.x + r:
		pos.x = b.position.x + r
		vel.x *= -wall_damping
# 	Right 
	elif pos.x > b.end.x - r:
		pos.x = b.end.x  - r
		vel.x *= -wall_damping
#	Top
	if pos.y < b.position.y + r:
		pos.y = b.position.y + r
		vel.y *= -wall_damping
#	Bottom
	elif pos.y > b.end.y - r:
		pos.y = b.end.y - r
		vel.y *= -wall_damping
	
	positions[i] = pos
	velocities[i] = vel

func poly6(r_sq:float) -> float:
	if r_sq >= h_sq:
		return 0.0
	var d = h_sq - r_sq
	return poly6_const * d * d * d
	
func calculate_density():
	for i in range(positions.size()):
		var dens = 0.0
		var pos_i = positions[i]
		for j in range(positions.size()):
			var offset = pos_i - positions[j]
			var r_sq = offset.length_squared()
			if r_sq >= h_sq: continue
			
			dens += mass * poly6(r_sq)
#		Add the density for i to the array and prevent 0 values
		densities[i] = maxf(dens, 0.00001)

func calculate_pressure():
#	Can try clamping negatives to 0 for "surface tension"
# 	pressures control the pulling and pushing force of the particle to hi/lo density
	var n = positions.size()
	for i in n:
		var pressure = stiffness * (densities[i] - rest_density)
#	    Play around with clamping to 0 vs not
		pressures[i] = pressure
	
func spiky_gradient(offset:Vector2, r: float) -> Vector2:
#	offset is pos[i] - pos[j] and r is offset.length
	if r <= 0.0 or r >= smoothing_scale:
		return Vector2.ZERO
	var q = smoothing_scale - r
	var magnitude = spiky_grad_const * q * q
	var direction = offset / r
	return direction * magnitude

func viscosity_lap(r:float) -> float:
	if r >= smoothing_scale: return 0.0
	return visc_lap_const * (smoothing_scale - r)

func calculate_forces():
	var n := positions.size()
	for i in n:
		var pos_i := positions[i]
		var vel_i := velocities[i]
		var press_i := pressures[i]
		var forcesComb = Vector2.ZERO
		for j in n:
			if j == i: continue
			
			var density_j := densities[j]
			if density_j <= 0.00001: continue
			
			var offset = pos_i - positions[j]
			var r_sq = offset.length_squared()
			if r_sq >= h_sq or r_sq == 0.0: continue
			
			var vel_diff = velocities[j] - vel_i
			
			#var r = offset.length()
			var r = sqrt(r_sq)
			
#			PRESSURE
			var pressure_force := (mass *(press_i + pressures[j]) /(2.0 * density_j))

			forcesComb -= (pressure_force * spiky_gradient(offset, r))

#			VISCOSITY
			var velocity_difference := (velocities[j] - vel_i)

			forcesComb += (viscosity *mass *velocity_difference /density_j * viscosity_lap(r))

		forces[i] = forcesComb
		forces[i] = forcesComb
		
func  integrate_forces(i:int, delta: float):
	var density = maxf(densities[i], 0.00001)
#	Acceleration from pressure + velocity
	var acceleration = (forces[i] / density + gravity)
	
	velocities[i] += acceleration * delta
	positions[i] += velocities[i] * delta
	
	
func _physics_process(delta):
	calculate_density()
	calculate_pressure()
	calculate_forces()
	for i in range(positions.size()):
		integrate_forces(i,delta)
		resolve_bounds(i)
	queue_redraw()
