#pragma once
// Dual-mode build support.
//
// When this app is co-compiled with companion_radio into a single firmware
// (the t1000e_dualmode_* envs, built with -D DUALMODE), its app-local class
// names would clash with companion_radio's identically named classes. This
// shim renames them for the repeater translation units only. It must be the
// first include in each simple_repeater .cpp so the macros are active before
// the class definitions are parsed.
//
// With no -D DUALMODE (the stock t1000e_repeater build) this header does
// nothing, so the normal repeater firmware is unaffected.
#ifdef DUALMODE
  #define MyMesh  RptMesh
  #define UITask  RptUITask
#endif
