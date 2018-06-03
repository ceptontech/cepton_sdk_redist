#pragma once

#include <stdint.h>
#include <cstdio>

#include <string>
#include <vector>

#include "cepton_sdk.h"

namespace cepton_sdk {

/*
  PCAP file reader.

  Interface is not thread-safe.
*/
class Capture {
 public:
  struct PacketHeader {
    uint32_t ip_v4;          // Host endian value, 0xC0A82020 = 192.168.32.32
    uint64_t serial_number;  // mac address last 3 bytes
    int64_t tv_usec;         // microseconds
    std::size_t data_size;
  };

 public:
  ~Capture();

  void clear_error_code() { m_error_code = CEPTON_SUCCESS; }
  CeptonSensorErrorCode error_code() const { return m_error_code; }

  std::string filename() const { return m_filename; }
  bool is_open() const { return m_fh != nullptr; }
  bool is_open_for_read() const { return (is_open() && m_is_read_mode); }
  bool is_open_for_write() const { return (is_open() && !m_is_read_mode); }
  bool open_for_read(std::string const &fname);
  bool open_for_write(std::string const &fname, bool append = true);
  void close();

  /*
    Returns time of start of file in microseconds since epoch.
  */
  int64_t start_time_usec() const { return m_start_time + m_timestamp_offset; }
  /*
    Returns time offset in microseconds since start of file.
  */
  int64_t current_offset_usec() const { return m_read_usec; }
  /*
    Returns length of file in microseconds.
  */
  int64_t total_offset_usec() const { return m_total_usec; }

  /*
    Seek to position in microseconds since start of file.

    Returns false if seek position is invalid.
  */
  bool seek(int64_t usec);

  /*
    Get next packet. Returns data length.
    If data length <= 0, then no data is returned.
    This happens at end of file, and if file is not open for reading.
  */
  CeptonSensorErrorCode next_packet(const PacketHeader **header,
                                    const uint8_t **data);

  /*
    Reset file reading.
  */
  void rewind() { read_file_header(); }

  /*
    Append packet to file.
  */
  bool append_packet(const PacketHeader *header, const uint8_t *data);

 private:
  void build_read_index();
  bool load_read_index(std::ifstream &f);
  void save_read_index(std::ofstream &f) const;
  void write_file_header();
  bool read_file_header();

 private:
  CeptonSensorErrorCode m_error_code = CEPTON_SUCCESS;
  FILE *m_fh = nullptr;
  std::string m_filename;
  bool m_is_read_mode;
  uint64_t m_read_pointer;
  uint64_t m_write_pointer;

  PacketHeader m_temp_header;
  std::vector<uint8_t> m_buffer;

  int64_t m_read_usec = 0;
  int64_t m_total_usec = 0;
  int64_t m_start_time = 0;
  int32_t m_timestamp_offset = 0;

  struct IndexFileHeader {
    std::size_t version = 0;
    uint64_t start_time;
    std::size_t n;
  };

  struct PacketIndex {
    int64_t time_usec;
    uint64_t read_ptr;
  };
  std::vector<PacketIndex> m_read_index;
};
}  // namespace cepton_sdk
