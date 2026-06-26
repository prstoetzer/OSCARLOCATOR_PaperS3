// ===========================================================================
//  oscar_types.h  -- shared type definitions for OSCARLOCATOR for PaperS3
//
//  These live in a header (not the .ino) on purpose. The Arduino IDE
//  auto-generates function prototypes and inserts them near the top of the
//  .ino, *before* any type defined in the .ino body. Functions whose
//  signatures use Proj / Btn / ResolvedMode would then have prototypes that
//  reference undefined types -> "does not name a type". Types pulled in via
//  #include are processed first, so putting them here fixes the ordering.
// ===========================================================================
#pragma once
#include <stdint.h>

// Projection mode actually in use (polar-auto resolves to North or South).
enum ResolvedMode { RM_POLAR_NORTH = 0, RM_POLAR_SOUTH, RM_QTH };

// Azimuthal-equidistant projection parameters + canvas geometry.
struct Proj {
  ResolvedMode rm;
  int    cx, cy, R;    // canvas geometry (disc centre + radius, px)
  double rmaxDeg;      // great-circle degrees mapped to the disc edge
};

// On-screen tappable button / tab.
struct Btn { int x, y, w, h; const char* label; };
