# Smooth SMP Transitions QA4 Papyrus integration

`Scripts/Source/ShapePhysicsLive.psc` declares the QA4 bridge. It exposes separate target queries and transition calls for GT and 3BA. `ShapePhysicsUnified.patch` contains the controller changes for OBody refresh, live transitions, and the MCM's accepted regional levels.

Apply the patch to the `Scripts/Source/ShapePhysicsUnified.psc` from the Shape Physics test configuration immediately before Live SMP was added. The table lists the SHA-256 checksums after normalizing line endings to LF.

| Controller | SHA-256 |
| --- | --- |
| Pre-Live Photo baseline | `e38857d8d9f5cb83c49e1faf350c64edd857d2cf78622b5823ee6c07b9234ed0` |
| QA4 after applying the patch | `f100672cd98c79e6eb18e3ce4296eb08f10fa17c8996d843ad06d8a7c0b3e68b` |

Normalize the baseline file to LF, then run `git -c core.autocrlf=false apply /path/to/ShapePhysicsUnified.patch` from the directory that contains `Scripts`. Compile the patched controller and bridge together with the Shape Physics Papyrus build inputs.
