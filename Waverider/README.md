# Waverider Forge

A hypersonic **waverider** design application built on the
[PicoGK](https://picogk.org) computational-geometry kernel.

You give it a **design Mach number** and an **altitude**. It solves the
real conical (Taylor–Maccoll) flow field, runs a constrained
volume-maximizing optimizer over the osculating-cone design space, reports the
aerodynamic performance, and produces a watertight 3D model (STL + OpenVDB)
that you can view, 3D-print, or push into a CFD/CAD pipeline.

> A waverider is a supersonic/hypersonic lifting body whose leading edge rides
> on its own attached bow shock. Because the high-pressure shock layer is
> trapped under the vehicle and cannot leak around the leading edge, waveriders
> achieve the highest lift-to-drag ratios known at hypersonic speeds.

## What it does (state of the art)

| Stage | Method |
|-------|--------|
| Atmosphere | 1976 U.S. Standard Atmosphere (to 86 km) + Sutherland viscosity |
| Inviscid flow field | **Taylor–Maccoll** conical-flow ODE, solved with RK4 |
| Geometry | **Sobieczky osculating-cone** inverse design with streamline tracing |
| Upper surface | freestream (streamwise) surface |
| Viscous drag | **Eckert reference-temperature** method (most accurate engineering model for waveriders) |
| Base drag | high-Mach base-pressure model |
| Leading edge | blunted, radius sized from **Sutton–Graves** stagnation heating |
| Optimization | maximize volumetric efficiency `τ = V^(2/3)/S_plan` subject to an L/D floor |

The optimizer's objective directly encodes the brief — *maximize useful volume
while preserving good flight characteristics*: it pushes for the most volume per
unit planform area while holding the lift-to-drag ratio at (by default) 90 % of
the best achievable value for the given flight condition.

## Build & run

Requires the **.NET 9 SDK** and a platform with a PicoGK native runtime
(Windows x64 or macOS arm64 — the binaries already ship under `../native/`).

```bash
# from the repository root
dotnet run --project Waverider -- 8 30          # Mach 8, 30 km (length prompts/defaults)
dotnet run --project Waverider -- 8 30 6        # Mach 8, 30 km, 6 m long

# or with named options
dotnet run --project Waverider -- --mach=10 --altitude-km=35 --length=8 --span=6 --view
```

Run with no arguments to be prompted interactively for Mach, altitude and length.
Positional arguments are `<mach> <altitudeKm> [lengthM]`.

### Fit inside a box

Instead of setting length/span directly, give a **length × width × height
envelope** and the optimizer returns the waverider that **encloses the most
volume inside that box** while holding L/D at the floor:

```bash
dotnet run --project Waverider -- 8 30 --box=6x5x1.2
```

It works by exploiting scale invariance: L/D and shape proportions don't depend
on absolute size, so the search ranks candidate *shapes*, scales each one up
until it just touches a box face, and maximizes the resulting absolute volume.
The report prints the target envelope and the achieved bounding box.

### Options

```
--mach=<M>             design Mach number
--altitude-km=<km>     altitude in kilometres        (or --altitude-m=<m>)
--length=<m>           vehicle length, metres                  (default 20)
--span=<m>             fix the full span (else the optimizer picks 0.5-1.0 x length)
--box=LxWxH            fit inside a length x width x height envelope (metres)
--ld-floor=<value>     absolute L/D floor for the optimizer
--ld-retention=<0..1>  L/D floor as a fraction of the max achievable (default 0.90)
--q-allow-mw=<MW/m^2>  allowable leading-edge stagnation heat flux  (default 5)
--sharp                sharp leading edge (no blunting)
--voxel-mm=<mm>        voxel size override
--out=<dir>            output directory                        (default ./output)
--view                 open the interactive PicoGK viewer
--sweep                run off-design Mach & AoA sweeps (tables + CSV)
--sweep-mach=min:max:count   custom Mach sweep range
--sweep-aoa=min:max:count    custom angle-of-attack sweep range (degrees)
```

### Off-design sweeps

Waveriders are designed for one Mach number — their shock only stays attached to
the leading edge at the design point, so L/D falls off as you move away from it.
`--sweep` holds the optimized geometry fixed and re-evaluates it with a
**Modified Newtonian** panel method across a range of Mach numbers (at the design
altitude) and angles of attack (at the design Mach):

```bash
dotnet run --project Waverider -- 8 30 --sweep
dotnet run --project Waverider -- 8 30 --sweep-mach=3:12:19 --sweep-aoa=-4:12:17
```

It prints two tables and writes `*_mach_sweep.csv` and `*_aoa_sweep.csv`
(columns: independent variable, altitude, L/D, C_L, C_D, lift, drag) for
plotting. Because the sweep uses Modified Newtonian consistently at every point,
its absolute L/D at the design Mach can differ slightly from the Taylor–Maccoll
design-point value — the meaningful output is the *trend* of L/D off-design.

## Output

* `output/waverider_M<mach>_<alt>km.stl` — watertight surface mesh (millimetres)
* `output/waverider_M<mach>_<alt>km.vdb` — OpenVDB voxel field
* A printed performance report: shock/cone angles, L/D, C_L, C_D, drag
  breakdown, volume, planform/wetted area, volumetric efficiency, leading-edge
  radius and stagnation heat flux.

## Source layout

| File | Responsibility |
|------|----------------|
| `Atmosphere.cs` | 1976 standard atmosphere → freestream state |
| `GasDynamics.cs` | isentropic / normal / oblique-shock relations |
| `TaylorMaccoll.cs` | conical flow solver, streamline shape, surface Cp |
| `WaveriderGeometry.cs` | osculating-cone construction + geometric metrics (pure math) |
| `AeroPerformance.cs` | pressure + reference-temperature viscous force integration |
| `Optimizer.cs` | constrained volume-maximizing search |
| `WaveriderBuilder.cs` | analytic surfaces → PicoGK mesh/voxels, LE blunting |
| `Program.cs` | CLI / interactive entry point, reporting, export, viewer |

The geometry and aerodynamics modules have **no PicoGK dependency**, so the
optimizer evaluates thousands of candidates cheaply; PicoGK is used only to
voxelize, mesh, export and visualize the final chosen design.

## Notes & roadmap

* The osculating-cone method assumes a constant conical shock angle across the
  span; where the base-plane shock curve is locally straight the local flow
  approaches a 2-D wedge.
* Performance is engineering-level (panel-method pressures + reference-
  temperature friction). Validate a finalized shape with CFD before committing
  to hardware.
* Off-design Mach/AoA sweeps are built in (`--sweep`, Modified Newtonian).
* Planned extensions: scramjet inlet-streamtube integration (reserve the
  captured tube for an engine), and direct surrogate-based multi-objective
  (volume vs L/D) Pareto search.
