# Setup — building and running the Waverider Forge app

This repository bundles the **PicoGK** geometry kernel *and* its native runtime
(`native/win-x64`, `native/osx-arm64`). The `Waverider` app's project file copies
the correct native binaries next to the executable automatically, so — unlike a
normal PicoGK install — you do **not** need to download or build the
"PicoGKRuntime" separately. You only need the .NET SDK.

## 1. Get the code

The app lives on the branch `claude/hypersonic-waverider-picogk-q56tn5`.

```bash
git clone https://github.com/aklothakis/PicoGKMine.git
cd PicoGKMine
git checkout claude/hypersonic-waverider-picogk-q56tn5
```

If you already have the repo locally:

```bash
git fetch origin
git checkout claude/hypersonic-waverider-picogk-q56tn5
```

## 2. Install .NET 9 SDK

Download from <https://dotnet.microsoft.com/download/dotnet/9.0>:

* **Windows** — the **x64** SDK installer.
* **Apple Silicon Mac (M1/M2/M3/M4)** — the **Arm64** SDK installer.

Verify (reopen the terminal first):

```bash
dotnet --version      # should print 9.x.x
```

> **Supported platforms:** Windows x64 and macOS Apple Silicon — the only native
> runtimes bundled in `native/`. Intel Macs and Linux require you to build the
> PicoGK runtime yourself.

## 3. Build & run

From the repository root:

```bash
# Mach 8 at 30 km altitude
dotnet run --project Waverider -- 8 30

# Interactive — prompts for Mach and altitude
dotnet run --project Waverider

# Named options, off-design sweep, and the 3D viewer
dotnet run --project Waverider -- --mach=10 --altitude-km=35 --length=24 --sweep --view
```

The first run restores packages and compiles (about a minute); later runs are fast.

### Graphical interface (Windows)

For a point-and-click interface instead of the command line, run the GUI
project (Windows only — it uses Windows Forms):

```bash
dotnet run --project WaveriderGui
```

Enter the Mach number, altitude and either a length or a "fit inside a box"
envelope, set the leading-edge / sweep / **voxel size** options (uncheck
"Auto voxel size" for a finer mesh and sharper detail), and click **Generate**.
The optimized waverider is rendered live in the preview pane (drag to orbit,
scroll to zoom) with the performance report beneath it. **Open 3D model (STL)**
opens the result in the Windows 3D viewer, and **Open output folder** shows the
exported files.

## 4. macOS only — clear Gatekeeper quarantine

The bundled `.dylib` files are not notarized, so macOS may block them
("cannot be opened because the developer cannot be verified"). Clear the flag
once, from the repository root, then re-run:

```bash
xattr -dr com.apple.quarantine .
```

Also make sure you installed the **Arm64** SDK (not the x64 one under Rosetta).

## 5. Output

Files are written to `./output/` (relative to where you ran the command):

| File | Description |
|------|-------------|
| `waverider_M<mach>_<alt>km.stl` | watertight surface mesh (open in any STL viewer) |
| `waverider_M<mach>_<alt>km.vdb` | OpenVDB voxel field |
| `..._mach_sweep.csv`, `..._aoa_sweep.csv` | off-design data (only with `--sweep`) |

A performance report and the sweep tables are printed to the console.

`--view` opens the interactive PicoGK viewer (drag to orbit); it needs a real
display/GPU, so run it on a desktop, not over plain SSH. Without `--view` the
files are still exported.

See `Waverider/README.md` for the full option list and the methodology.

## 6. Troubleshooting

| Symptom | Fix |
|---------|-----|
| `dotnet: command not found` | Install the SDK; reopen the terminal. |
| `Failed to load PicoGK library` | macOS: run the `xattr` command above; confirm Arm64 SDK. Windows: confirm x64. See `PicoGK.log` in your Documents folder. |
| Viewer errors on a headless machine | Drop `--view`; STL/VDB still export. |
| Compile errors | Report them — this is the first build on a supported platform. |
