/**
 * Sample code for error callback usage.
 */
#include <cepton_sdk_api.hpp>

int main(int argc, char** argv) {
  // Initialize
  cepton_sdk::api::SensorErrorCallback error_callback;
  CEPTON_CHECK_ERROR(error_callback.listen(
      [&](cepton_sdk::SensorHandle handle,
          const cepton_sdk::SensorError& error) { throw error; }));
  CEPTON_CHECK_ERROR(cepton_sdk::initialize(
      CEPTON_SDK_VERSION, cepton_sdk::create_options(),
      error_callback.global_on_callback, &error_callback));

  // Deinitialize
  cepton_sdk::deinitialize().ignore();
}
