# Faster HDT-SMP (FSMP)

## Shape Physics GT Live SMP QA

This fork contains the Shape Physics experimental patch for FSMP 4.1.1. The patch was added on November 24, 2026 and is based on the [official 4.1.1 build commit](https://github.com/DaymareOn/hdtSMP64/commit/2215ec3ba09adf0843a3291ed0437a6ca6d385e2).

When Shape Physics requests a compatible GT profile for the same attached body, FSMP blends chest spring stiffness and damping in the running simulation. It keeps the same physics system, pose, and bone velocities. The source XML provides the new spring values. Shape Physics requests a half-second transition during automatic shape-based updates.

This QA build targets the player's GT body with the automatic Shape Physics profile active. Its scope is linear and angular spring stiffness and damping. The local AVX build passed the XML validator and Skyrim AE 1.6.1170 reached the main menu with it loaded. In-game motion testing is pending.

The archive from the QA workflow contains the replacement `hdtsmp64.dll`, the source commit, a SHA-256 checksum, and the FSMP license texts. A game test also needs the paired Shape Physics QA scripts and the original FSMP installation with its dependencies. The Shape Physics integration will be available on Nexus after the in-game check. [GitHub Actions](https://github.com/ddumaa/hdtSMP64/actions/workflows/shape-physics-gt-live.yml)

The modified FSMP code is distributed under GPLv3 with the upstream exceptions. [License](LICENSE) · [Exceptions](EXCEPTIONS)

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
