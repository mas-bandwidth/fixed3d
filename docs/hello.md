# Hello Fixed3D {#hello}
The Fixed3D distribution includes a Hello World unit test written in C. The test
creates a large static ground box and a small dynamic box. This code does not
contain any graphics. All you will see is text output in the console showing
the box's position over time.

This is a good example of how to get up and running with Fixed3D.

## Creating a World
Every Fixed3D program begins with the creation of a world object.
The world is the physics hub that manages memory, objects, and simulation.
The world is represented by an opaque handle called `b3WorldId`.

It is easy to create a Fixed3D world. First, create the world definition:

```c
b3WorldDef worldDef = b3DefaultWorldDef();
```

The world definition is a temporary object you can create on the stack. The function
`b3DefaultWorldDef()` populates the world definition with default values. This is necessary
because C does not have constructors and zero-initializing `b3WorldDef` is not appropriate.

Fixed3D has no built-in concept of *up*. The gravity vector is a `b3Vec3` and can point in
any direction. Convention in Fixed3D examples uses +Y as the up axis. The default gravity
is already `{0, -10, 0}`, but it can be set explicitly:

```c
worldDef.gravity = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( -10.0f ), B3_FIX( 0.0f ) };
```

Fixed3D stores every scalar quantity — positions, velocities, densities, time
steps — as `b3Fixed` (Q48.16 fixed point in an `int64_t`). The `B3_FIX` macro
converts a float literal at compile time. Assigning a bare float literal like
`-10.0f` to a `b3Fixed` field compiles but truncates to the integer `-10` raw —
off by a factor of 65536 — so always wrap literals in `B3_FIX`.

Now create the world:

```c
b3WorldId worldId = b3CreateWorld(&worldDef);
```

World creation copies all the data it needs from the definition, so the definition can
go out of scope immediately afterward.

## Creating a Ground Box
Bodies are built using the following steps:
1. Define a body with position, type, etc.
2. Use the world id to create the body.
3. Build a hull shape with the desired extents.
4. Create the shape on the body.

For step 1, create the ground body definition and set its initial position:

```c
b3BodyDef groundBodyDef = b3DefaultBodyDef();
groundBodyDef.position = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( -10.0f ), B3_FIX( 0.0f ) };
```

For step 2, use the world id to create the ground body. Bodies are static by default,
meaning they have zero mass, never move, and do not collide with other static bodies.

```c
b3BodyId groundId = b3CreateBody(worldId, &groundBodyDef);
```

Notice that `worldId` is passed by value. Ids are small structures and are always passed
by value.

For steps 3 and 4, build a box hull and attach it. Fixed3D uses convex hulls for box
shapes. The `b3MakeBoxHull` helper takes three **half-extents** (hx, hy, hz), so the
ground slab below is 100 units wide in X, 20 units tall in Y, and 100 units deep in Z:

```c
b3BoxHull groundBox = b3MakeBoxHull(B3_FIX( 50.0f ), B3_FIX( 10.0f ), B3_FIX( 50.0f ));

b3ShapeDef groundShapeDef = b3DefaultShapeDef();
b3CreateHullShape(groundId, &groundShapeDef, &groundBox.base);
```

The `.base` field holds the `b3HullData` that `b3CreateHullShape` expects. Fixed3D copies
the hull data into a shared internal database, so `groundBox` does not need to outlive
the call. Do not call `b3DestroyHull` on a `b3BoxHull`; it is stack-allocated.

Fixed3D is tuned for meters, kilograms, and seconds, so the extents above are in meters.
The engine works best when objects are sized like real-world objects (a barrel is roughly
1 m tall). Simulating glaciers or dust particles would push the limits of the fixed-point
resolution (1/65536 of a meter).

Every shape must have a parent body, even static shapes. You can attach multiple shapes
to one body. A shape's world transform is inherited from its parent body; there is no
independent shape transform.

## Creating a Dynamic Body
Creating a dynamic body follows the same steps. The key difference is setting the body
type to `b3_dynamicBody` and giving the shape a non-zero density.

```c
b3BodyDef bodyDef = b3DefaultBodyDef();
bodyDef.type = b3_dynamicBody;
bodyDef.position = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 4.0f ), B3_FIX( 0.0f ) };
b3BodyId bodyId = b3CreateBody(worldId, &bodyDef);
```

> **Caution**:
> You must set the body type to `b3_dynamicBody` if you want the body to
> move in response to forces such as gravity.

Create a unit cube hull and a shape definition with density and friction:

```c
b3BoxHull dynamicBox = b3MakeCubeHull(B3_FIX( 1.0f ));

b3ShapeDef shapeDef = b3DefaultShapeDef();
shapeDef.density = B3_FIX( 1.0f );
shapeDef.baseMaterial.friction = B3_FIX( 0.3f );

b3CreateHullShape(bodyId, &shapeDef, &dynamicBox.base);
```

`b3MakeCubeHull(r)` is a convenience that produces a cube with half-extent `r` on all
three axes, equivalent to `b3MakeBoxHull(r, r, r)`.

> **Caution**:
> A dynamic body should have at least one shape with a non-zero density.
> Otherwise you will get unexpected behavior.

That completes initialization. We are now ready to simulate.

## Simulating the World
Fixed3D uses a numerical integrator that advances the simulation by discrete time steps.
A fixed time step of 1/60 seconds (60 Hz) is recommended for most games. Avoid tying
the time step to your frame rate; a variable time step produces variable results that
are hard to debug.

```c
b3Fixed timeStep = B3_FIX( 1.0f / 60.0f );
```

In addition to integration, Fixed3D uses a constraint solver. Fixed3D advances through the
time step in several *sub-steps*, giving each constraint multiple chances to react. Four
sub-steps is the suggested value:

```c
int subStepCount = 4;
```

At 60 Hz with 4 sub-steps the constraints run at 240 Hz internally. More sub-steps
improve accuracy at the cost of performance.

The simulation loop calls `b3World_Step` once per game tick:

```c
for (int i = 0; i < 90; ++i)
{
    b3World_Step(worldId, timeStep, subStepCount);

    b3Vec3 position = b3Body_GetPosition(bodyId);
    b3Quat rotation = b3Body_GetRotation(bodyId);

    printf("%4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f\n",
           b3FixToDouble(position.x), b3FixToDouble(position.y), b3FixToDouble(position.z),
           b3FixToDouble(rotation.v.x), b3FixToDouble(rotation.v.y), b3FixToDouble(rotation.v.z),
           b3FixToDouble(rotation.s));
}
```

Note the `b3FixToDouble` conversions: a `b3Fixed` is an `int64_t`, so passing
one straight to a `%f` format specifier is undefined behavior. Convert at the
printing (or rendering) boundary.

`b3Body_GetPosition` returns a `b3Vec3` with the body origin in world space.
`b3Body_GetRotation` returns a `b3Quat` — a unit quaternion stored as a vector part
`q.v` (x, y, z) and a scalar part `q.s`. There is no single angle to extract as there
was in 2D; orientation in 3D requires the full quaternion. To convert to an axis-angle
representation use `b3GetAxisAngle`:

```c
b3Fixed angle;
b3Vec3 axis = b3GetAxisAngle(&angle, rotation);
```

`angle` is in radians and `axis` is the unit rotation axis. Use
`b3MakeMatrixFromQuat` when you need a 3×3 rotation matrix, for example to feed a
renderer.

The output should show the box falling from y = 4 and coming to rest on the ground at
approximately y = 1 (the box half-height sits at y = 0 + 1 after the ground surface
at y = 0):

```
0.00 4.00 0.00 ...
0.00 3.99 0.00 ...
0.00 3.98 0.00 ...
...
0.00 1.25 0.00 ...
0.00 1.13 0.00 ...
0.00 1.01 0.00 ...
```

For advice on managing a fixed simulation rate alongside a variable render rate, see
[Fix Your Timestep!](https://gafferongames.com/post/fix_your_timestep/).

## Multithreading (optional)
By default Fixed3D runs single-threaded. The `b3DefaultWorldDef` leaves `workerCount`
at 1 and the task callbacks null, which is fine for getting started. When performance
matters, Fixed3D can drive a task system. Supply `workerCount` plus `enqueueTask`,
`finishTask`, and `userTaskContext` on the world definition before calling
`b3CreateWorld`. See the Foundations page for details.

## Cleanup
When you are done with the simulation, destroy the world:

```c
b3DestroyWorld(worldId);
```

This efficiently destroys all bodies, shapes, and joints in the simulation.
