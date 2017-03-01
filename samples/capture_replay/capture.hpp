#pragma once

#include <cstdio>
#include <stdint.h>
#include <mutex>
#include <vector>

namespace cepton {
// Cepton specific captures
class Capture {
public:
  struct PacketHeader {
    uint32_t ip_v4; // Host endian value, 0xC0A82020 = 192.168.32.32
    uint64_t serial_number; // mac address last 3 bytes
    int64_t tv_usec; // microseconds
    std::size_t data_size;
  };

  Capture() : m_fh(nullptr), m_buffer(nullptr), m_buf_size(0) {}
  ~Capture() {
    close(); // Just in case
    if (m_buffer) {
      delete m_buffer;
      m_buffer = nullptr;
    }
  }

  bool open_for_read(std::string const &fname);
  bool is_valid_pcap() const { return m_is_valid; }
  int64_t start_time_usec() const { return m_start_time; }
  int64_t current_offset_usec() const { return m_read_usec; } // Offset from the start

  bool open_for_write(std::string const &fname, bool append = true);
  void close() {
    std::lock_guard<std::mutex> guard(get_lock());
    if (m_fh) {
      fclose(m_fh);
      m_fh = nullptr;
    }
  }

  bool is_open() const { return m_fh != nullptr; }
  //------------------------
  // Reading functions
  void build_read_index();
  bool seek(int64_t usec);
  bool seek_rel(int64_t usec) {
    if (m_read_usec + usec >= 0)
      seek(m_read_usec + usec);
    else
      rewind();
    return true;
  }
  // Returns length
  unsigned next_packet(PacketHeader const **pkt_header, uint8_t const **pkt_data);
  void rewind() { // Reset the reading, next call to next_package returns the first one
    read_file_header();
  }
  //------------------------
  // Writing functions

  // Use "current system time" here
  bool append_packet(PacketHeader const *hdr, uint8_t const *pkt_data, size_t data_len);

  std::mutex &get_lock() { return m_lock; }
private:
  FILE *m_fh;
  uint64_t m_read_pointer;
  uint64_t m_write_pointer;

  PacketHeader m_temp_header; // Returned header for each "next_packet" call
  uint8_t *m_buffer;
  uint32_t m_buf_size;

  int64_t m_read_usec;
  int64_t m_start_time = 0;
  std::mutex m_lock;
  struct PacketIndex {
    int64_t time_usec;
    uint64_t read_ptr;
  };

  std::vector<PacketIndex> m_read_index;
  bool m_is_valid;

  void write_file_header();
  void read_file_header();
};

}
