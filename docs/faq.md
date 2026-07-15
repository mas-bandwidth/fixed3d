# FAQ

## What is Fixed3D?
Fixed3D is a fixed-point fork of Box3D, a feature rich 3D rigid body physics engine written in C17 by Erin Catto. Box3D has
been used in games and game engines as a 3D counterpart to the well-established Box2D engine.

Fixed3D uses the [MIT license](https://en.wikipedia.org/wiki/MIT_License) and can be used free of charge. Credit
should be included if possible. Support is [appreciated](https://github.com/sponsors/erincatto).

## What platforms does Fixed3D support?
Fixed3D is developed using C17. It is portable and can be compiled for any platform with a conforming C17 compiler.

Erin Catto maintains the C version of upstream Box3D. Community ports and bindings for other languages are not officially supported.

## Who makes it?
Erin Catto is the creator and primary author of Box3D, the engine Fixed3D is forked from. Box3D is an open source project, and accepts community feedback via
[GitHub Issues](https://github.com/erincatto/box3d/issues) and
[GitHub Discussions](https://github.com/erincatto/box3d/discussions).

Fixed3D — this fork — is maintained by Glenn Fiedler and Rowan, and ports new
upstream Box3D work across as it lands. Issues specific to Fixed3D are welcome
on [Fixed3D Issues](https://github.com/mas-bandwidth/fixed3d/issues).

## How do I get help?
You should read the documentation and the rest of this FAQ first. Also, you should study the examples included in the source
distribution. For Fixed3D-specific problems, file a
[Fixed3D issue](https://github.com/mas-bandwidth/fixed3d/issues). For general
Box3D questions you can visit the upstream [Discord](https://discord.gg/NKYgCBP) —
but please keep fork-specific questions out of Erin's channels.

Please do not message or email Erin Catto directly for support, and do not
report Fixed3D bugs to him — this fork's behavior is not his to debug.

## Documentation

### Why isn't a feature documented?
If you grab the latest code from the git main branch you will likely find features that are not documented in the manual.
New features are added to the manual after they are mature and a new point release is imminent. However, all major features
added to Fixed3D are accompanied by example code in the samples application to test the feature and show the intended usage.

## Prerequisites

### Programming
You should have a working knowledge of C before you use Fixed3D. You should understand functions, structures, and pointers.
There are plenty of resources on the web for learning C. You should also understand your development environment:
compilation, linking, and debugging.

### Math and Physics
You should have a basic knowledge of rigid bodies, force, torque, and impulses. In 3D you will also encounter quaternions
for orientation and 3x3 inertia tensors for rotational dynamics. If you come across a math or physics concept you don't
understand, please read about it on Wikipedia. Visit this [page](https://box2d.org/publications/) if you want a deeper
knowledge of the algorithms used in Fixed3D.

## API

### What units does Fixed3D use?
Fixed3D is tuned for meters-kilograms-seconds (MKS). This is recommended as the unit system for your game. However, you may
use different units if you are careful. Call `b3SetLengthUnitsPerMeter()` at startup to change the length unit.

### What coordinate system does Fixed3D use?
Fixed3D has no built-in notion of up. The gravity vector in `b3WorldDef` can point in any direction. The default is
`(0, -10, 0)` (negative Y is down) but you can set it to whatever suits your application.

### Why don't you use this awesome language?
Fixed3D is designed to be portable and easy to wrap with other languages, so I decided to use C17. I used C17 to get
support for atomics.

### Can I use Fixed3D in a DLL?
Yes. See the CMake option `BUILD_SHARED_LIBS`.

### Is Fixed3D thread-safe?
No. Fixed3D will likely never be fully thread-safe from the outside. Fixed3D has a large API and trying to make such an API
thread-safe would have a large performance and complexity impact. However, you can call read-only functions from multiple
threads. For example, all the spatial query functions are read-only.

Fixed3D does use multithreading internally during `b3World_Step`. You supply your own task system via the `enqueueTask` and
`finishTask` callbacks in `b3WorldDef`.

## Build Issues

### Why doesn't my code compile and/or link?
There are many reasons why a build can go bad. Here are a few that have come up:
* Using old Fixed3D headers with new code
* Not linking the Fixed3D library with your application
* Using old project files that don't include some new source files

## Rendering

### What are Fixed3D's rendering capabilities?
Fixed3D is only a physics engine. How you draw stuff is up to you.

### But the samples application draws stuff
Visualization is very important for debugging collision and physics. The samples application helps test Fixed3D and gives
you examples of how to use Fixed3D. The samples are not part of the Fixed3D library.

### How do I draw shapes?
Fill out a `b3DebugDraw` struct with your drawing callbacks and call `b3World_Draw(worldId, &draw, maskBits)`.
The mask bits let you filter which shape categories are drawn.

## Accuracy
Fixed3D uses approximate methods for a few reasons.
* Performance
* Some differential equations don't have known solutions
* Some constraints cannot be determined uniquely

What this means is that constraints are not perfectly rigid and sometimes you will see some bounce even when the restitution
is zero. Fixed3D uses [Gauss-Seidel](https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method) to approximately solve
constraints. Fixed3D also uses [Semi-implicit Euler](https://en.wikipedia.org/wiki/Semi-implicit_Euler_method) to
approximately solve the differential equations. Fixed3D also does not have exact collision between dynamic shapes. Slow
moving shapes may have small overlap for a few time steps. In extreme stacking scenarios, shapes may have sustained
overlap.

## Making Games

### Tile / Voxel Based Environments
Using many boxes for terrain may not work well because box-like characters can get snagged on internal corners. Fixed3D
provides capsules and convex hulls that may work better for characters. Consider the character mover API
(`b3World_CastMover`, `b3World_CollideMover`) for smooth first/third person movement.

### Asteroid Type Coordinate Systems
Fixed3D does not have any support for coordinate frame wrapping. You would likely need to customize Fixed3D for this purpose.

## Determinism

### Is Fixed3D deterministic?
For the same input Fixed3D will reproduce any simulation. Fixed3D does not use any random numbers nor base any computation
on random events (such as timers, etc).

Fixed3D is also deterministic under multithreading. A simulation using two threads will give the same result as eight
threads. This is the largest part of the determinism story, and it is Erin Catto's engineering, inherited from Box3D:
making a multithreaded solver produce identical results regardless of thread count or scheduling is a substantial,
ongoing piece of work (simulation order pinned to creation order, deterministic event ordering, and more), and it must
be actively preserved by every change — which is why the determinism unit test runs on every pull request in both trees.

The *math layer* of cross-platform determinism is where this fork differs. Fixed3D is deterministic by construction
there: the simulation is pure integer math (Q48.16 fixed point), so there are no floating-point flags to police. Box3D
is *also* cross-platform deterministic at the math layer — it achieves this with floating-point discipline
(`-ffp-contract=off`, consistent IEEE 754 arithmetic across compilers and ISAs). Fixed point changes how that one layer
is achieved, not whether — and the threading layer above it, the bulk of the work, is identical in both trees.

However, Fixed3D does not have rollback determinism. There is no mechanism to set a world back to a prior state and then
resume simulation expecting identical results. Fixed3D caches a lot of internal state to improve simulation stability and
performance.

### But I really want fixed point
Box3D does not support fixed-point math — upstream's FAQ notes that it is slower and more tedious to develop,
which is why Erin chose not to use it. This fork exists to measure exactly that: the entire simulation is Q48.16 fixed
point, and it comes out about 2× slower (see the README). He was right on both counts. If determinism is all you want,
float Box3D already provides it.

## What are the common mistakes made by new users?
* Using non-metric units instead of meters
* Expecting Fixed3D to give pixel-perfect results
* Testing their code in release mode (always use Debug for testing — it enables assertions and validation)
* Not learning C before using Fixed3D
* Confusing b2 (Box2D) and b3 (Box3D) symbols when reading documentation — Fixed3D keeps the b3 prefix so Box3D code ports unchanged
