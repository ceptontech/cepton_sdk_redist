/** Define macros.
 */
#ifdef CEPTON_SDK_COMPILING  // Export

#ifdef _MSC_VER
#define CEPTON_EXPORT __declspec(dllexport)
#elif __GNUC__
#define CEPTON_EXPORT __attribute__((visibility("default")))
#else
#define CEPTON_EXPORT
#endif

#elif defined(CEPTON_SDK_STATIC)  // Import static

#define CEPTON_EXPORT

#else  // Import shared

#ifdef _MSC_VER
#define CEPTON_EXPORT __declspec(dllimport)
#else
#define CEPTON_EXPORT
#endif

#endif

#if defined(_MSC_VER)
#define CEPTON_DEPRECATED __declspec(deprecated)
#elif defined(__GNUC__)
#define CEPTON_DEPRECATED __attribute__((deprecated))
#else
#define CEPTON_DEPRECATED
#endif