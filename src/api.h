#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define HammeringTaskNew_DLLIMPORT __declspec(dllimport)
#  define HammeringTaskNew_DLLEXPORT __declspec(dllexport)
#  define HammeringTaskNew_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define HammeringTaskNew_DLLIMPORT __attribute__((visibility("default")))
#    define HammeringTaskNew_DLLEXPORT __attribute__((visibility("default")))
#    define HammeringTaskNew_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define HammeringTaskNew_DLLIMPORT
#    define HammeringTaskNew_DLLEXPORT
#    define HammeringTaskNew_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef HammeringTaskNew_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define HammeringTaskNew_DLLAPI
#  define HammeringTaskNew_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef HammeringTaskNew_EXPORTS
#    define HammeringTaskNew_DLLAPI HammeringTaskNew_DLLEXPORT
#  else
#    define HammeringTaskNew_DLLAPI HammeringTaskNew_DLLIMPORT
#  endif // HammeringTaskNew_EXPORTS
#  define HammeringTaskNew_LOCAL HammeringTaskNew_DLLLOCAL
#endif // HammeringTaskNew_STATIC