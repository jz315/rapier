# Studio Rapier analytic profile extension

This crate is part of the Physics Intuition Studio Rapier fork. It extends the
pinned official Rapier 2D build with an analytic, closed profile shape made
from line segments and circular arcs.

It is not a second physics engine or a standalone WASM package. The official
TypeScript/WASM binding workspace links it into the same Rapier world and emits
one Worker-only ES module and WASM resource.

Supported contacts are analytic profile against Rapier balls, cuboids, and
other analytic profiles.

The upstream Rapier and Parry sources retain their own Apache-2.0 licenses;
the generated package copies those license texts and records pinned versions
and hashes in its manifest.
