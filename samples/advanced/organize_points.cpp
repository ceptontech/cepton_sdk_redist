/*******************************************************************
 **                                                               **
 **  Copyright(C) 2019 Cepton Technologies. All Rights Reserved.  **
 **  Contact: https://www.cepton.com                              **
 **                                                               **
 **  Sample code which opens a cepton sensor pcap file, organizes **
 **  the points and continuously saves the most recent organized  **
 **  points to a frame to a cvs file "organized_cloud.cvs         **
 *******************************************************************/

#include <cepton_sdk_util.hpp>
#include <cepton_sdk/capture.hpp>
#include <cepton_sdk_api.hpp>

using namespace cepton_sdk::util;

int main(int argc, char** argv) {
  if (argc < 2) return -1;
  const std::string capture_path = argv[1];

  // Initialize sdk
  auto options = cepton_sdk::create_options();
  options.control_flags |= CEPTON_SDK_CONTROL_DISABLE_NETWORK;
  options.frame.mode = CEPTON_SDK_FRAME_COVER;
  CEPTON_CHECK_ERROR(cepton_sdk::api::initialize(options));

  cepton_sdk::SensorInformation sensor_info;

  OrganizedCloud organized_cloud;
  std::ofstream os;

  cepton_sdk::Capture m_capture;
  CEPTON_CHECK_ERROR(m_capture.open_for_read(capture_path));

  CEPTON_CHECK_ERROR(
      cepton_sdk_set_control_flags(CEPTON_SDK_CONTROL_DISABLE_NETWORK,
                                   CEPTON_SDK_CONTROL_DISABLE_NETWORK));
  CEPTON_CHECK_ERROR(cepton_sdk_clear());

  // Listen for points
  cepton_sdk::api::SensorImageFrameCallback callback;
  CEPTON_CHECK_ERROR(callback.initialize());
  callback.listen([&](cepton_sdk::SensorHandle handle, std::size_t n_points,
                     const cepton_sdk::SensorImagePoint* c_image_points) {

    cepton_sdk::get_sensor_information(handle,sensor_info);

    std::printf("Received %i points from sensor %lli\n", static_cast<int>(n_points),
                static_cast<long long>(handle));

    Organizer organizer(sensor_info);

    organizer.organize_points(n_points,
                              sensor_info.return_count,
                              c_image_points,
                              organized_cloud);

    os.open("organize_cloud.csv");
    for (const auto& point : organized_cloud.points)
    {
      if (point.valid)
      {
        float x = 0;
        float y = 0;
        float z = 0;
        cepton_sdk::util::convert_image_point_to_point(
            point.image_x, point.image_z, point.distance, x,
            y, z);

        os << x << "," << y << "," << z << "\n";
      }
    }
    os.close();
  });

  while (true) {
    cepton_sdk::Capture::PacketHeader header;
    const uint8_t* data;
    CEPTON_CHECK_ERROR(m_capture.next_packet(header, data));

    const cepton_sdk::SensorHandle handle =
        static_cast<cepton_sdk::SensorHandle>(header.ip_v4) |
        CEPTON_SENSOR_HANDLE_FLAG_MOCK;
    CEPTON_CHECK_ERROR(cepton_sdk_mock_network_receive(
        handle, header.timestamp, data, static_cast<size_t>(header.data_size)));
  }
}
