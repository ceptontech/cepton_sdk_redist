#pragma once

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "cepton_sdk.hpp"

namespace cepton_sdk {

/*
  PCAP file reader.

  Interface is not thread-safe.
*/
class Capture {
 public:
  struct PacketHeader {
    uint32_t ip_v4;
    int64_t timestamp;
    int data_size;
  };

 private:
  struct IndexFileHeader {
    std::size_t version = 0;
    uint64_t start_time;
    std::size_t n;
  };

  struct PacketIndex {
    int64_t position;
    uint64_t pointer;
  };

  struct PacketData {
    int id;
    bool mf;
    int off;
    int len;

    int n_fragments;
    PacketHeader header;
    std::vector<uint8_t> buffer;

    PacketData() { reset(); }

    void reset() {
      id = -1;
      mf = false;
      off = 0;
      len = 0;

      n_fragments = 0;
      header = {};
      buffer.clear();
    }
  };

 public:
  ~Capture();

  std::string filename() const { return m_filename; }
  bool is_open() const { return m_stream.is_open(); }
  bool is_open_for_read() const { return (is_open() && m_is_read_mode); }
  bool is_open_for_write() const { return (is_open() && !m_is_read_mode); }
  SensorError open_for_read(std::string const &filename);
  SensorError open_for_write(std::string const &filename, bool append = true);
  void close();

  int64_t start_time() const { return m_start_time + m_timestamp_offset; }
  int64_t position() const { return m_position; }
  int64_t length() const { return m_length; }

  SensorError seek(int64_t position);

  SensorError next_packet(PacketHeader &header, const uint8_t *&data);
  SensorError append_packet(const PacketHeader &header, const uint8_t *data);

  void rewind() { read_file_header(); }

 private:
  SensorError open_for_read_impl(const std::string &filename);
  SensorError open_for_write_impl(const std::string &filename, bool append);
  SensorError build_read_index();
  SensorError load_read_index(std::ifstream &f);
  SensorError save_read_index(std::ofstream &f) const;
  SensorError write_file_header();
  SensorError read_file_header();

  SensorError next_packet_impl(bool &success, PacketHeader &header,
                               const uint8_t *&data);

 private:
  std::fstream m_stream;
  std::string m_filename;
  bool m_is_read_mode;
  int64_t m_read_ptr;
  int64_t m_write_ptr;
  int64_t m_timestamp_offset = 0;
  int64_t m_start_time = 0;
  int64_t m_position = 0;
  int64_t m_length = 0;
  std::vector<PacketIndex> m_read_index;

  std::map<uint32_t, PacketData> m_packets;
};
}  // namespace cepton_sdk
