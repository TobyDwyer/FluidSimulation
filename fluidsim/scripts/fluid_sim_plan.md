# SPH Fluid Sim — Build Plan

A phase-by-phase plan for getting from "circles in a box" to a working 2D SPH fluid.
Each phase has something you can see and test before moving on.

---

## Phase 0 — The architectural decision

**You will need to stop using RigidBody2D.** This is the one big change, and it's worth
understanding why before you do anything else.

SPH works by letting particles *overlap*. Each particle has a smoothing radius `h` that is
2–3× the spacing between particles, and it feels every neighbour inside that radius. The
forces come from a density estimate, not from contact.

RigidBody2D is built on the opposite assumption. Its solver treats overlap as an error state
and spends every frame pushing bodies apart. So:

- It fights your pressure forces — you push particles together, it shoves them apart
- You don't control integration order, substepping, or damping
- Contact-pair broadphase for thousands of bodies is far slower than array maths
- You can't sub-step without also sub-stepping the whole physics server

### What to replace it with

Parallel arrays on your Node2D, one entry per particle:

| Array | Type | Purpose |
|---|---|---|
| `positions` | `PackedVector2Array` | where each particle is |
| `velocities` | `PackedVector2Array` | current velocity |
| `forces` | `PackedVector2Array` | accumulated force this step |
| `densities` | `PackedFloat32Array` | density estimate |
| `pressures` | `PackedFloat32Array` | derived from density |

`Packed*Array` types are tightly packed in memory rather than being arrays of Variants, so
they're substantially faster for this kind of per-element numeric work.

### What you keep

- Your `_draw()` loop — change `p.position` to `positions[i]` and it works unchanged
- Your container **as a visual only** (`draw_rect`) — boundaries become code, not physics bodies
- `_physics_process` as your update tick

Your StaticBody2D container goes away. Boundary handling becomes four `if` statements.

---

## Phase 1 — Custom integration loop

**Add:** gravity, integration, boundary bounce. No fluid behaviour yet.

The loop each step:

1. Zero the force array
2. Add gravity to every particle: `force += mass * gravity`
3. Integrate velocity: `velocity += (force / mass) * dt`
4. Integrate position: `position += velocity * dt`
5. Resolve boundaries

Use **semi-implicit Euler** — velocity updated first, then position uses the *new* velocity.
This is one line different from plain Euler and dramatically more stable. It's the standard
choice for SPH.

### Boundary handling

For each wall, if a particle is outside: clamp its position back to the boundary and reverse
the perpendicular velocity component, scaled by a damping factor.

```
if pos.x < min_x:
    pos.x = min_x
    vel.x *= -boundary_damping
```

Damping must be **less than 1**, or walls inject energy and the sim never settles.

### Test

Spawn particles in a loose cloud, run it.

**Expected:** they fall, hit the floor, bounce lower each time, settle into a flat overlapping
layer along the bottom — all at essentially the same y. They will all pile into the same
place because nothing pushes them apart yet. That's correct at this stage.

**Failure modes:**

| Symptom | Cause |
|---|---|
| Particles jitter on the floor forever | boundary damping ≥ 1, or position not clamped before velocity flip |
| Particles fall upward | Godot's Y axis points **down** — gravity is `+Y`, not `−Y` |
| Particles escape the box | integrating position before clamping, or checking bounds before moving |

---

## Phase 2 — Density estimation

**Add:** the poly6 kernel and a density pass.

For each particle `i`, sum over every particle `j` within `h`:

```
density[i] = Σ mass * W_poly6(distance(i,j), h)
```

**Include the particle itself** (where `r = 0`). Leaving self out is one of the most common
bugs — surface particles come out with absurdly low density and the sim explodes there.

### The kernel (2D — these constants are not the 3D ones)

```
W_poly6(r, h) = (4 / (π · h⁸)) · (h² − r²)³        for 0 ≤ r ≤ h
              = 0                                   otherwise
```

Almost every SPH tutorial online is written for 3D. The 3D poly6 constant is
`315/(64π h⁹)`. If you copy it into a 2D sim your densities are wrong by a large and
non-obvious factor. Watch for this in every kernel.

**Optimisation:** compare `r²` against `h²` directly rather than taking a square root for the
radius check. You only need the actual distance when the kernel requires `r` itself.

### Test

Colour particles by density instead of white. Map density to a colour ramp — dark blue for
low, bright cyan for high.

**Expected:** a falling cloud shows roughly uniform low density. Once piled on the floor, the
interior of the pile is bright and the outer edge is noticeably darker. Surface particles
legitimately read about half the density of interior ones, because half their neighbourhood
is empty. This is real, not a bug, and it's why free surfaces are the hard part of SPH.

**Failure modes:**

| Symptom | Cause |
|---|---|
| All densities zero | radius check inverted, or `h` smaller than particle spacing |
| Density uniform regardless of clumping | not actually iterating neighbours |
| Density identical for all particles including surface | self-contribution dominating — `h` too small |

---

## Phase 3 — Pressure from density

**Add:** an equation of state. One line, no new loop — fold it into the end of the density pass.

```
pressure[i] = stiffness * (density[i] - rest_density)
```

### Choosing rest density — measure it, don't guess

This is the parameter people waste the most time on. Don't pick a number off the internet.

1. Lay your particles out on a **regular grid** at your intended spacing
2. Run the density calculation once
3. Print the mean density of the *interior* particles (ignore the edges)
4. That value is your `rest_density`

The mathematical shortcut: a uniform grid at spacing `d` gives roughly `density ≈ mass / d²`,
because the kernel integrates to 1 over its area. So if you want a round `rest_density`, set
`mass = rest_density * d²`.

The absolute scale of mass and density doesn't matter. What matters is `stiffness` *relative*
to them.

**Clamp pressure to ≥ 0 initially.** Negative pressure pulls particles together, which gives
you cohesion and surface tension — nice later, unstable now.

---

## Phase 4 — Pressure force

**Add:** a second loop (after densities are complete for all particles) applying pressure forces.

```
F_pressure[i] = −Σ mass · (pressure[i] + pressure[j]) / (2 · density[j]) · ∇W_spiky(r_ij, h)
```

The averaged `(P_i + P_j)/2` term is what makes the force symmetric, so momentum is conserved
and the fluid doesn't drift on its own.

### Use the spiky gradient, not poly6

```
∇W_spiky(r, h) = (−10 / (π · h⁵)) · (h − r)² · r̂
```

where `r̂` is the unit vector from `i` to `j`.

The poly6 gradient goes to **zero as r → 0**. If you use it for pressure, particles that get
very close stop repelling each other and collapse into tight clusters. Spiky's gradient grows
as they approach, which is exactly what you need. This is the single most important kernel
choice in the whole sim.

Guard against `r = 0` — the unit vector is undefined. Skip the pair, or nudge one particle.

### Test

Drop a blob of particles into the box.

**Expected:** the blob resists compression, spreads out under its own weight, and settles into
a body with a roughly **flat surface** and roughly **even particle spacing** throughout. It
should look like liquid, not like a pile of sand.

**Failure modes:**

| Symptom | Cause |
|---|---|
| Violent explosion on frame 1 | `stiffness` too high for your `dt` — lower stiffness or add substeps |
| Particles collapse to a single point | force sign flipped (pressure pushes *apart*) |
| Particles pair up into tight clumps | using poly6 gradient instead of spiky |
| Slow uncontrolled drift in one direction | asymmetric force — check the `(P_i + P_j)/2` term |
| Sim stable but fluid compresses into the floor | `stiffness` too low |

---

## Phase 5 — Viscosity

**Add:** a viscosity term in the same loop as pressure.

```
F_viscosity[i] = μ · Σ mass · (velocity[j] − velocity[i]) / density[j] · ∇²W_visc(r_ij, h)
```

```
∇²W_visc(r, h) = (40 / (π · h⁵)) · (h − r)
```

Viscosity is a *velocity-difference* force — it drags each particle toward the average
velocity of its neighbours. It is what stops the sim looking like a spray of independent dots.

It's also your main stability lever. A sim that's marginally unstable will often settle down
with more viscosity.

### Test

Sweep `μ` and watch the character change.

| μ | Expected behaviour |
|---|---|
| 0 | chaotic, spray-like, individual particles fly off the surface |
| low | water — splashy, waves, fast settling |
| medium | oil — cohesive, visible sloshing, slower settling |
| high | honey — barely flows, deforms slowly under gravity |

If you can't get a clear difference across that range, the viscosity force isn't being applied.

---

## Phase 6 — Spatial hash

**Add:** neighbour lookup. This is pure optimisation — the simulation should look *identical*
afterward.

Phases 2–5 are O(n²): every particle checks every other particle. That's fine up to roughly
500 particles and hopeless beyond it.

The fix:

1. Choose cell size = `h` exactly
2. Each step, clear a `Dictionary` and re-bucket every particle under key
   `Vector2i(floor(pos / h))`
3. To find neighbours of a particle, check its own cell plus the 8 surrounding cells

With cell size `h`, a particle's entire smoothing radius is guaranteed to fall within those
9 cells. Larger cells means checking more irrelevant particles; smaller means checking more
cells. `h` is the sweet spot.

Rebuild the hash **every step** — particles have moved.

### Test

Run the same scene with brute force and with the hash, same seed.

**Expected:** visually identical behaviour, and a large framerate jump — at 1000+ particles
the difference should be dramatic. If behaviour *changed*, your cell lookup is missing
neighbours near cell boundaries.

---

## Phase 7 — Stability and timestep

SPH with stiff pressure needs a small timestep. Godot's `_physics_process` runs at 60 Hz
(`dt ≈ 0.0167`), which is **far too coarse** for a stiff fluid.

**Sub-step inside your physics tick:** run the whole simulation loop N times per frame with
`dt = 0.0167 / N`. Start with N = 4 and raise it if the sim is unstable.

Rough stability guide (CFL condition): `dt` must be small enough that no particle travels
more than a fraction of `h` in one step. If your fastest particle moves more than about
`0.4 · h` per step, you need more substeps.

**Other stability levers, roughly in order of what to reach for first:**

1. More substeps (always safe, costs CPU)
2. More viscosity (changes the look)
3. Lower stiffness (fluid becomes more compressible)
4. Velocity clamping (blunt, but stops one bad particle destroying the sim)

---

## Parameter reference

### Recipe — derive a self-consistent set

1. Pick `h` — the smoothing radius. Everything scales from this.
2. Pick spacing `d ≈ h / 2.5`. This gives roughly 15–20 neighbours per particle, the
   standard target for 2D SPH. Fewer than ~10 gives noisy densities; more than ~30 is
   wasted computation.
3. Set `mass = 1`.
4. **Measure** `rest_density` on a grid at spacing `d` (see Phase 3).
5. Tune `stiffness` until the fluid holds its volume without exploding.
6. Tune `μ` for the look you want.

### Starting values

Pixel units, Godot coordinates (**Y is down**):

| Parameter | Start with | Notes |
|---|---|---|
| `h` (smoothing radius) | 16 | main scale knob |
| particle spacing | 6–7 | ≈ h/2.5 |
| `particle_size` (drawn) | 3 | purely visual — ~half spacing looks right |
| `mass` | 1.0 | arbitrary; density scales with it |
| `rest_density` | *measure it* | expect ≈ `mass / spacing²` |
| `stiffness` | start low, raise | most sensitive parameter |
| `μ` (viscosity) | start mid-range | raise if unstable |
| `gravity` | `Vector2(0, 980)` | +Y is down; 980 px/s² ≈ 9.8 m/s² at 100px/m |
| `boundary_damping` | 0.5 | must be < 1 |
| substeps | 4 | raise before touching anything else |

**Known-working reference set** (from Schuermann's 2D implementation, linked below —
`h = 16`, `mass = 2.5`, `rest_density = 300`, `stiffness = 2000`, `μ = 200`,
`dt = 0.0007`, `boundary_damping = 0.5`):

These numbers are **internally coupled**. They work together in that implementation.
Don't take one value from this set and one from elsewhere — that's how you get a sim that
explodes for no visible reason. Either adopt the whole set or derive your own with the recipe
above. Note that `dt = 0.0007` implies roughly 24 substeps per 60 Hz frame.

---

## Test scenarios

Run these in order. Each isolates a different failure.

### 1. Hydrostatic rest — the stability test

Fill the bottom half of the box with particles on a regular grid. Let it run.

**Expected:** minor initial settling, then it goes **still**. Surface flat and level. Density
roughly uniform in the interior, and increasing slightly with depth.

**This is the test most sims fail.** If it never stops moving, you have an energy source
somewhere — usually boundary damping, an asymmetric force, or too large a timestep. Fix it
here before trying anything dynamic. A sim that can't sit still will never look like water.

### 2. Dam break — the benchmark

Fill the left third of the box, floor to two-thirds height. Release at t=0.

**Expected:** the column collapses, a wave front rushes right along the floor, hits the far
wall, climbs it, curls over and breaks backward, sloshes back and forth with decreasing
amplitude, and settles flat after a few seconds.

This is the standard SPH benchmark and it exercises everything — pressure, viscosity,
boundaries, free surface. If the dam break looks right, your sim is working.

### 3. Droplet drop

A small round blob dropped from height into still fluid.

**Expected:** impact crater, a splash crown, secondary droplets, ripples spreading outward,
surface returning to flat.

Good test of whether viscosity is tuned sensibly — too low and it shatters into spray, too
high and it lands like a blob of putty with no splash at all.

### Particle counts

| Purpose | Count |
|---|---|
| Debugging logic — small enough to print and inspect | 50–100 |
| Visual testing, brute force | 300–500 |
| After spatial hash | 2000–5000 |

Start small. It is far easier to spot a wrong force with 50 particles than with 2000.

---

## Pitfall checklist

Things that will cost you an evening each:

- [ ] **Godot's Y axis points down.** Gravity is `+Y`. Every SPH reference you read assumes `+Y` is up.
- [ ] **2D kernel constants are not 3D kernel constants.** Most tutorials online are 3D.
- [ ] **Include the particle itself** in its own density sum.
- [ ] **Spiky gradient for pressure**, poly6 for density, viscosity kernel for viscosity. Three different kernels, deliberately.
- [ ] **Two separate passes.** All densities must be computed before *any* pressure force is applied. One fused loop gives you wrong results that look almost plausible.
- [ ] **Guard `r = 0`.** Two particles at the same position produce a divide-by-zero in the unit vector.
- [ ] **Rebuild the spatial hash every step.**
- [ ] **Boundary damping < 1**, always.
- [ ] **Clamp positions before flipping velocity**, not after.
- [ ] **Symmetric pressure force** — `(P_i + P_j)/2`, not just `P_i`.

---

## File structure

Don't build this as one long script. The natural split follows the phases above, and it maps
cleanly onto Godot's class system.

```
res://
├── main.gd                  Node2D      — scene setup, owns the tick, wires things together
├── sim/
│   ├── sim_config.gd        Resource    — every tunable parameter
│   ├── kernels.gd           static      — poly6, spiky gradient, viscosity laplacian
│   ├── spatial_hash.gd      RefCounted  — neighbour lookup
│   └── fluid_solver.gd      RefCounted  — the particle arrays and the step() function
└── render/
    └── fluid_renderer.gd    Node2D      — _draw() and nothing else
```

### What goes where, and why

**`sim_config.gd` — extend `Resource`, give it `class_name SimConfig`**

All your `@export` parameters live here: `h`, `mass`, `rest_density`, `stiffness`, `viscosity`,
`gravity`, `boundary_damping`, `substeps`, `bounds_size`.

Making it a `Resource` rather than a plain script is the highest-value decision in this list.
You can save parameter sets as `.tres` files in the filesystem — `water.tres`, `honey.tres`,
`unstable_test.tres` — edit them in the Inspector without touching code, and swap between them
by dragging a different file into an export slot. When you're tuning stiffness and viscosity
for the twentieth time, this is what saves you.

**`kernels.gd` — `class_name Kernels`, all `static func`**

Pure maths: input numbers, output numbers, no state. Static functions mean you call
`Kernels.poly6(r, h)` from anywhere without instantiating anything.

Keeping these isolated matters because kernel constants are the most likely thing to be
subtly wrong, and they're the easiest thing to unit-test — you can verify a kernel integrates
to roughly 1 over its area without running the sim at all.

**`spatial_hash.gd` — extend `RefCounted`, `class_name SpatialHash`**

`build(positions, cell_size)` and `get_neighbours(index)`. Nothing else.

Isolating this is what lets you do the Phase 6 test properly: keep a brute-force path
alongside it and swap between them with a flag. Same results, different speed — that's your
proof the hash is correct.

**`fluid_solver.gd` — extend `RefCounted`, `class_name FluidSolver`**

Owns the particle arrays and does the work. Roughly:

- `positions`, `velocities`, `forces`, `densities`, `pressures`
- `step(dt)` — one substep: compute densities, compute forces, integrate, resolve boundaries
- private methods for each of those stages

`RefCounted` rather than `Node` because this has no business being in the scene tree — it has
no transform, no children, no `_process`. It's a data structure with methods. Skipping `Node`
avoids the scene-tree overhead entirely.

**`fluid_renderer.gd` — extend `Node2D`**

Holds a reference to the solver, reads `solver.positions` and `solver.densities`, draws them.
Never writes anything.

This separation is what lets you change the look without touching the physics — swap circles
for metaballs, add velocity vectors for debugging, colour by density instead of speed — with
zero risk of breaking the simulation.

**`main.gd` — extend `Node2D`**

Creates the solver from the config, spawns the initial particle layout, and runs the tick:

```
func _physics_process(delta):
    for i in config.substeps:
        solver.step(delta / config.substeps)
    renderer.queue_redraw()
```

That's essentially the whole file. If `main.gd` starts growing kernel maths or neighbour
loops, something has leaked out of where it belongs.

### The rule of thumb

The split that matters most is **simulation state vs. rendering**. Everything else is tidiness,
but that one is structural — it's what makes the Phase 0 change from RigidBody2D to arrays a
contained edit rather than a rewrite, and it's what lets you test the physics by printing
numbers instead of squinting at circles.

Build it as one file if you want while you're finding your feet, but split `_draw()` out
from the solver early.

---

## Sources

- [Implementing SPH in 2D — Lucas V. Schuermann](https://lucasschuermann.com/writing/implementing-sph-in-2d) — 2D kernel constants and a known-working parameter set
- [Particle-Based Fluid Simulation — Lucas V. Schuermann](https://lucasschuermann.com/writing/particle-based-fluid-simulation) — derivation and background
- [Smoothed Particle Hydrodynamics — UIUC CS418](https://cs418.cs.illinois.edu/website/text/sph.html) — course notes on kernels and the algorithm
- [Particle-based Fluids — CMU 15-467](https://www.cs.cmu.edu/~scoros/cs15467-s16/lectures/11-fluids2.pdf) — lecture slides
