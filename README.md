# Faster HDT-SMP (FSMP)

## Shape Physics – Smooth SMP Transitions

QA4 adds smooth transitions for **GT SMP** and **3BA SMP** to FSMP 4.1.1. When the player's automatic Shape Physics preset changes within the same compatible XML family, FSMP blends the new parameters over half a second in the running simulation. It keeps the existing model, physics system, and velocities. [FSMP 4.1.1 build commit](https://github.com/DaymareOn/hdtSMP64/commit/2215ec3ba09adf0843a3291ed0437a6ca6d385e2)

- **GT SMP** blends spring stiffness, damping, and compatible travel limits for the main body and the slot 60 auxiliary system.
- **3BA SMP** blends the same spring parameters and travel limits, plus nonlinear spring coefficients and chest bone gravity. A full SMP profile can cover the chest, glutes, abdomen, and thighs. With a mixed setup, each zone follows its active physics engine.

The first attachment and changes to a different profile family use the usual XML load. The integration uses the existing Shape Physics morph scores and regional XML generator. The local QA4 AVX build targeted Skyrim AE 1.6.1170 and passed three FSMP test suites. The XML comparisons covered 298 profiles. In-game QA4 motion testing is pending.

### Pair with Shape Physics In-Game Geometry QA

Use the In-Game Geometry QA add-on with the In-Game Geometry QA main package from Shape Physics commit `1123f3f`. The add-on contains the QA4 native DLL and bridge API v3, with its Papyrus controller adapted for in-game body measurement. Install the add-on after the main package in MO2 so its controller takes priority. [Main package](https://github.com/ddumaa/shape-physics/blob/1123f3fe3ef0d29a4eb8a5b063313fe042ef5fb0/outputs/Shape%20Physics%201.1.0%20-%20In-Game%20Geometry%20QA.zip) · [Add-on package](https://github.com/ddumaa/shape-physics/blob/1123f3fe3ef0d29a4eb8a5b063313fe042ef5fb0/outputs/Shape%20Physics%20-%20Smooth%20SMP%20Transitions%20-%20In-Game%20Geometry%20QA.zip)

To rebuild the pair, use the Shape Physics archive builder with the Photo Physics QA main archive and pass its matching Photo Physics QA add-on through `--smooth` and the destination through `--smooth-output`. The builder adapts and compiles both controllers while retaining the native DLL from the add-on. [Archive builder](https://github.com/ddumaa/shape-physics/blob/1123f3fe3ef0d29a4eb8a5b063313fe042ef5fb0/qa/runtime-geometry/Build-Runtime-Geometry-Archive.py)

### Source and build

The native changes are in `src`. The paired Papyrus sources include the bridge declarations and the patch against the exact pre-Live Photo controller. The bridge reports API version 3. [Papyrus bridge](shape-physics/qa4/Scripts/Source/ShapePhysicsLive.psc) · [Controller patch](shape-physics/qa4/ShapePhysicsUnified.patch) · [Controller baseline](shape-physics/qa4/README.md)

Use Visual Studio 2022, CMake, and vcpkg with `VCPKG_ROOT` pointing to your vcpkg installation. Run these commands from the fork's root in PowerShell.

```powershell
git submodule update --init --recursive
$pluginRoot = Join-Path (Get-Location).Path 'out/build/vs2022-windows-avx/plugins'
New-Item -ItemType Directory -Force -Path (Join-Path $pluginRoot 'SKSE/Plugins') | Out-Null
cmake --preset vs2022-windows-avx "-DCompiledPluginsPath=$pluginRoot" -DPROJECT_BUILD_INFO=shape-physics-live-smp-qa4 -DBUILD_VALIDATOR_TESTS=ON -DBUILD_PATTERN_TESTS=ON -DBUILD_CONFIG_TESTS=ON
cmake --build out/build/vs2022-windows-avx --config Release
ctest --test-dir out/build/vs2022-windows-avx -C Release --output-on-failure --no-tests=error
```

Use the resulting DLL with the matching Shape Physics scripts, the original FSMP installation and its runtime dependencies. The modified FSMP code is distributed under GPLv3 with the upstream exceptions. [License](LICENSE) · [Exceptions](EXCEPTIONS) · [Earlier GT-only QA1](https://github.com/ddumaa/hdtSMP64/tree/shape-physics-gt-live-qa1)

## Upstream FSMP

Skinned-mesh physics for Skyrim **SE / AE / VR** — the actively-maintained continuation of HDT-SMP.

Maintained by DaydreamingDay / DaymareOn. Fork of [Karonar1's hdtSMP64](https://github.com/Karonar1/hdtSMP64), itself a fork of the [version](https://github.com/aers/hdtSMP64) by aers, from the [original code](https://github.com/HydrogensaysHDT/hdt-skyrimse-mods) by HydrogensaysHDT.

Mod page: <https://www.nexusmods.com/skyrimspecialedition/mods/57339>

## Documentation

Most documentation lives in the wikis; both have a sidebar linking every page.

- **Players** — the [FSMP wiki](https://github.com/DaymareOn/hdtSMP64/wiki) covers installation, configuration, the in-game config menu (via the [SKSE Menu Framework](https://www.nexusmods.com/skyrimspecialedition/mods/120352)), console commands, solving problems, and the changelog.
- **Mod authors** — the [SMP Modder Guide](https://github.com/DaymareOn/FSMP-Validator/wiki) covers authoring the physics XML and meshes, next to the XSD/Schematron schemas that define a valid file. See also `smp report` (validate a whole load order from the console) and the DynamicHDT Papyrus API (control physics from scripts), both in the FSMP wiki.
- **Developers** — building FSMP, the `smp_replay` benchmark, and the code analyses are in the wiki's "Building FSMP" section. Build steps: [How to compile your own FSMP](https://github.com/DaymareOn/hdtSMP64/wiki/6-%E2%80%90-How-to-compile-your-own-FSMP).

## Changes

For the full, maintained changelog, see the [wiki Changelog](https://github.com/DaymareOn/hdtSMP64/wiki).

## Known issues

For current known bugs, see the [Nexus bugs tab](https://nexusmods.com/skyrimspecialedition/mods/57339?tab=bugs) and [GitHub issues](https://github.com/DaymareOn/hdtSMP64/issues).

## Credits

See the [Hall of Fame](https://github.com/DaymareOn/hdtSMP64/wiki/01-%E2%80%90-Hall-of-Fame) for the full list of contributors and acknowledgements.
