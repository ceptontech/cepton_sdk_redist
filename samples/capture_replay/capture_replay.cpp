#include "capture_replay.hpp"

#include <cepton_sdk.h>

namespace cepton {

bool CaptureReplay::load_capture(std::string const &fname) {
  if (m_pcap.is_open()) m_pcap.close();

  if (!m_pcap.open_for_read(fname))
    return false; // Failed to open main pcap file, fatal

  const Capture::PacketHeader *pkt_hdr;
  const uint8_t *pkt_data;
  int len = m_pcap.next_packet(&pkt_hdr, &pkt_data);
  if (len <= 0) {
    close();
    return false;
  }
  serial_number = pkt_hdr->serial_number;
  if (serial_number == 0)
    serial_number = (pkt_hdr->ip_v4 >> 24) - 32; // NOTICE This is pre-swap
  //cfg.set(pkt->x_scale, pkt->y_scale, pkt->distance_scale, pkt->focal_length);

  m_pcap.rewind(); // Start from the beginning.

  internal_resume();
  load_config(fname);
  
  return true;
}

void CaptureReplay::internal_resume() {
  auto since_epoch = std::chrono::high_resolution_clock::now().time_since_epoch();

  // Apply offset
  start_usec = std::chrono::duration_cast<std::chrono::microseconds>(since_epoch).count();
  start_usec -= m_pcap.current_offset_usec();
  terminate_feeder.store(false);

  m_pcap_feeder = new std::thread([this]() { feed_pcap(); });
}

bool CaptureReplay::load_config(std::string const &fname) {
  // Load the .cfg file
  return false;
}

void CaptureReplay::feed_pcap() {
  const Capture::PacketHeader *pkt_hdr;
  const uint8_t *pkt_data;

  for (; !terminate_feeder;) {
    int len = m_pcap.next_packet(&pkt_hdr, &pkt_data);

    while (len > 0 && !terminate_feeder) {

      auto since_epoch = std::chrono::high_resolution_clock::now().time_since_epoch();
      int64_t ts_usec = std::chrono::duration_cast<std::chrono::microseconds>(since_epoch).count();

      int64_t tdelta = m_pcap.current_offset_usec() - (ts_usec - start_usec);

      if (tdelta > 0 && tdelta < 1000000) {
        std::this_thread::sleep_for(std::chrono::microseconds(tdelta));
      }
      else if (tdelta <= 0) {
        // catchup, do nothing here
      }
      else {
        m_pcap.seek(ts_usec - start_usec);
      }

      uint8_t mac[6];
      mac[0] = 0x18;
      mac[1] = 0x12;
      mac[2] = 0x12;
      mac[3] = (uint8_t)((pkt_hdr->serial_number >> 16) & 0xFF);
      mac[4] = (uint8_t)((pkt_hdr->serial_number >> 8) & 0xFF);
      mac[5] = (uint8_t)(pkt_hdr->serial_number & 0xFF);
      before_packet();
      
      cepton_sdk_mock_network_receive((uint64_t)pkt_hdr->ip_v4, mac,
        pkt_data, pkt_hdr->data_size);

      len = m_pcap.next_packet(&pkt_hdr, &pkt_data);
    }
    printf(terminate_feeder ? "Terminated\n" : "Rewind!\n");
    if (terminate_feeder) break;

    m_pcap.rewind();

    auto since_epoch = std::chrono::high_resolution_clock::now().time_since_epoch();

    start_usec = std::chrono::duration_cast<std::chrono::microseconds>(since_epoch).count();
  }
}

void CaptureReplay::skip_backward(float sec) {
  int64_t delta = (int64_t)(-sec * 1e6f);
  m_pcap.seek_rel(delta);
  start_usec -= delta;     
}

void CaptureReplay::skip_forward(float sec) {
  int64_t delta = (int64_t)(sec * 1e6f);
  m_pcap.seek_rel(delta);
  start_usec -= delta;
}
}
