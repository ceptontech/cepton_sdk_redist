/** Define macros.
 */
#ifdef CEPTON_SDK_COMPILING // Export

#ifdef _MSC_VER
#define EXPORT __declspec(dllexport)
#elif __GNUC__
#define EXPORT __attribute__((visibility("default")))
#else
#define EXPORT
#endif

#elif defined(CEPTON_SDK_STATIC) // Import static

#define EXPORT

#else // Import shared

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