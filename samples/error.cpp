/**
 * Sample code for error callback usage.
 */
#include <iostream>

#include <cepton_sdk_api.hpp>

int main(int argc, char** argv) {
  // Initialize
  cepton_sdk::api::SensorErrorCallback error_callback;
  error_callback.listen(
      0, [&](cepton_sdk::SensorHandle handle,
             const cepton_sdk::SensorError& error) { throw error; });
  cepton_sdk::api::check_error(cepton_sdk::initialize(
      CEPTON_SDK_VERSION, cepton_sdk::create_options(),
      error_callback.global_on_callback, &error_callback));
}
