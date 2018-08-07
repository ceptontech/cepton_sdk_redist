/** Define macros.
 */

#ifdef COMPILING

#ifdef _MSC_VER
#define EXPORT __declspec(dllexport)
#elif __GNUC__
#define EXPORT __attribute__((visibility("default")))
#else
#define EXPORT
#endif

#else

#ifdef _MSC_VER
#define EXPORT __declspec(dllimport)
#else
#define EXPORT
#endif

#endif

#if defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#elif defined(__GNUC__)
#define DEPRECATED __attribute__((deprecated))
#else
#define DEPRECATED
#endif