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
  // v1.17.0 made both halves' NodePrefs polymorphic (ConfigSerializer base with
  // a virtual structure()). Same class name + vtable = vague linkage: the linker
  // keeps ONE vtable for the whole binary, so one half serializes its prefs
  // through the OTHER half's structure() against the wrong memory layout.
  // Renaming the repeater's class gives each half its own vtable. CommonCLI.h
  // carries an identical #define for TUs that include it without this shim.
  #define NodePrefs RptNodePrefs
#endif
