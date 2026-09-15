extends Node2D

@export var particle_size: float = 10
@export var num_particles: int = 20

# The starting pos params for the particles.
var xmin: int = -100
var ymin: int = -100
var xmax: int = 100
var ymax: int = 100
# Container postion and size
var container_pos: Vector2 = Vector2(0, 0)
var container_size: Vector2 = Vector2(1000, 600)

var particles: Array[RigidBody2D] = []

func _ready():
	make_container()
	for i in range(num_particles):
		spawn_particle()
	
func spawn_particle():
	var particle = RigidBody2D.new()
	# Give each pariticle a slightly random starting point
	particle.position = Vector2(randf_range(xmin,xmax), randf_range(ymin,ymax))
	
	# give the particles collisions
	var collision = CollisionShape2D.new()
	var circle = CircleShape2D.new()
	
	circle.radius = particle_size
	collision.shape = circle
	
	particle.add_child(collision)
	# add completed particle with shape and collision 
	add_child(particle)
	# store in array
	particles.append(particle)
		
#	Create the physics body for the container for the fluid
func make_container():
#	Create the static body object this is the parent of the shape
	var container = StaticBody2D.new()
#	Use collision polygon so objects can exist inside.
	var poly = CollisionPolygon2D.new()
	var h = container_size / 2
	
#	set the bounds of all the corners of the polygon
	poly.polygon = PackedVector2Array([
		Vector2(-h.x, -h.y),   # top-left
		Vector2( h.x, -h.y),   # top-right
		Vector2( h.x,  h.y),   # bottom-right
		Vector2(-h.x,  h.y),   # bottom-left
	])
	poly.build_mode = CollisionPolygon2D.BUILD_SEGMENTS
	
#	Add the poly shape to the conatiner body
	container.add_child(poly)
	add_child(container)
	
func _draw():
#	Rect2 starts in the top left so we have to maek ti so that it is offset by half 
	draw_rect(Rect2(container_pos-container_size/2,container_size),Color.WHITE,false,2.0)
	for p in particles:
		draw_circle(p.position, particle_size,Color.WHITE)

func _physics_process(delta):
		queue_redraw()
