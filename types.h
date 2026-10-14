#pragma once
// Kept in a header so Arduino's auto-generated prototypes can see them.
struct PV { float x, y, z, i; };  // projected vertex: screen xy, depth 0..32767, intensity x256
struct Box { int x0, y0, x1, y1; };
