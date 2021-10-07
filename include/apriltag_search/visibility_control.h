
#ifndef APRILTAG_SEARCHER_VISIBILITY_CONTROL_H
#define APRILTAG_SEARCHER_VISIBILITY_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CPP_EXPORT __attribute__((dllexport))
#define CPP_IMPORT __attribute__((dllimport))
#else
#define CPP_EXPORT __declspec(dllexport)
#define CPP_IMPORT __declspec(dllimport)
#endif
#ifdef CPP_BUILDING_DLL
#define CPP_PUBLIC CPP_EXPORT
#else
#define CPP_PUBLIC CPP_IMPORT
#endif
#define CPP_PUBLIC_TYPE CPP_PUBLIC
#define CPP_LOCAL
#else
#define CPP_EXPORT __attribute__((visibility("default")))
#define CPP_IMPORT
#if __GNUC__ >= 4
#define CPP_PUBLIC __attribute__((visibility("default")))
#define CPP_LOCAL __attribute__((visibility("hidden")))
#else
#define CPP_PUBLIC
#define CPP_LOCAL
#endif
#define CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // APRILTAG_SEARCHER_VISIBILITY_CONTROL_H
