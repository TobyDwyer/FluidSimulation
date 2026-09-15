extends Node2D

@export var particle_size: float = 10
@export var num_particles: int = 20
@export var gravity: Vector2 = Vector2(0, 980)
@export var wall_damping: float = 0.5

# The starting pos params for the particles.
var xmin: int = -100
var ymin: int = -100
var xmax: int = 100
var ymax: int = 100

# Container postion and size
var container_pos: Vector2 = Vector2(0, 0)
var container_size: Vector2 = Vector2(1000, 600)

# Simulation state. One entry per particle — index i is the same
# particle in every array. More arrays get added as you go:
# forces, densities, pressures.
var positions: PackedVector2Array = PackedVector2Array()
var velocities: PackedVector2Array = PackedVector2Array()

func _ready():
	for i in range(num_particles):
		spawn_particle()

func spawn_particle():
	# Give each particle a slightly random starting point
	positions.append(Vector2(randf_range(xmin, xmax), randf_range(ymin, ymax)))
	# Starts at rest. Nothing moves it yet — that arrives with the integrator.
	velocities.append(Vector2.ZERO)

func _draw():
	draw_rect(bounds(), Color.WHITE, false, 2.0)
	for pos in positions:
		draw_circle(pos, particle_size, Color.WHITE)

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

func  apply_gravity(i:int, delta: float):
	velocities[i] += gravity * delta
	positions[i] += velocities[i] * delta
	



func _physics_process(delta):
	for i in range(positions.size()):
		apply_gravity(i,delta)
		resolve_bounds(i)
	queue_redraw()
