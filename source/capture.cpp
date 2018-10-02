#include "cepton_sdk/capture.hpp"

#include <cstring>

#include <algorithm>
#include <fstream>
#include <string>

#include "cepton_sdk_api.hpp"

namespace cepton_sdk {

SensorError check_file(std::ios &stream) {
  SensorError error;
  if (!stream) error = CEPTON_ERROR_FILE_IO;
  stream.clear();
  return error;
}

#define CHECK_FILE(stream)                     \
  {                                            \
    auto internal_error = check_file(stream);  \
    if (internal_error) return internal_error; \
  }

bool is_eof(std::istream &stream) {
  bool result = false;
  if (stream.peek() == EOF) result = true;
  stream.clear();
  return result;
}

template <typename T>
std::istream &read_values(std::istream &f, T *const values, int n) {
  return f.read((char *)values, n * sizeof(T));
}

template <typename T>
std::istream &read_value(std::istream &f, T &value) {
  return read_values(f, &value, 1);
}

template <typename T>
std::ostream &write_values(std::ostream &f, const T *const values, int n) {
  return f.write((char *)values, n * sizeof(T));
}

template <typename T>
std::ostream &write_value(std::ostream &f, const T &value) {
  return write_values(f, &value, 1);
}

template <typename T>
T swap_endian(const T &value) {
  if (sizeof(T) == 1) return value;

  // Not super fast, but simple.
  T result;
  const uint8_t *const value_bytes = (const uint8_t *)&value;
  uint8_t *const result_bytes = (uint8_t *)&result;
  for (int i = 0; i < sizeof(T); ++i) {
    result_bytes[i] = value_bytes[sizeof(T) - i - 1];
  }
  return result;
}

template <typename T>
void swap_endian_inplace(T &value) {
  value = swap_endian(value);
}

template <int i, typename T>
T get_bit_mask() {
  return T(1) << T(i);
}

template <typename T>
T get_bit_mask(int i) {
  return T(1) << T(i);
}

template <typename T>
bool get_bit(const T &value, int i) {
  return value & get_bit_mask<T>(i);
}

template <int start, int size, typename T>
T get_bit_field_mask() {
  return ((T(1) << T(size)) - T(1)) << T(start);
}

template <int start, int size, typename T>
T get_bit_field(const T &value) {
  return (value & get_bit_field_mask<start, size, T>()) >> T(start);
}

template <typename T>
std::string get_bit_str(const T &value) {
  std::string result(8 * sizeof(T), '0');
  for (int i = 0; i < 8 * sizeof(T); ++i) {
    if (get_bit(value, i)) result[i] = '1';
  }
  return result;
}

template <typename T>
std::string get_hex_str(const T &value) {
  std::string result(2 * sizeof(T), '0');
  const uint8_t *const value_bytes = (const uint8_t *)&value;
  for (int i = 0; i < sizeof(T); ++i) {
    std::sprintf(&result[2 * i], "%02X", value_bytes[i]);
  }
  return result;
}

// -----------------------------------------------------------------------------
// PCAP
// -----------------------------------------------------------------------------
const uint32_t PCAP_MAGIC = 0xA1B2C3D4;
const uint16_t PCAP_VERSION_MAJOR = 2;
const uint16_t PCAP_VERSION_MINOR = 4;
const uint32_t PCAP_LINKTYPE_ETHERNET = 1;

const uint16_t PROTOCOL_V4 = 0x0008;

const uint8_t IP_HL_VER = 0x45;
const uint8_t IP_PROTOCOL_UDP = 0x11;

#pragma pack(1)
struct PCAPFileHeader {
  uint32_t magic;          // magic number
  uint16_t version_major;  // major version number
  uint16_t version_minor;  // minor version number
  uint32_t thiszone;       // gmt to local correction
  uint32_t sigfigs;        // accuracy of timestamps
  uint32_t snaplen;        // max length saved portion of each packet, in octets
  uint32_t linktype;       // data link type
};
const int pcap_file_header_size = sizeof(PCAPFileHeader);

#pragma pack(1)
struct PCAPRecordHeader {
  uint32_t ts_sec;    // timestamp seconds
  uint32_t ts_usec;   // timestamp microseconds
  uint32_t incl_len;  // number of octets of packet saved in file
  uint32_t orig_len;  // actual length of packet
};
const int pcap_record_header_size = sizeof(PCAPRecordHeader);

enum {
  IP_FLAG_MF = 1 << 0,  // more fragments
  IP_FLAG_DF = 1 << 1,  // don't fragment
};

// Big endian
#pragma pack(1)
struct IPHeader {
  uint8_t hl_ver;     // header length + version
  uint8_t tos;        // type of service
  uint16_t len;       // total length
  uint16_t id;        // identification
  uint16_t frag_off;  // flags + fragment offset
  uint8_t ttl;        // time to live
  uint8_t p;          // protocol
  uint16_t sum;       // checksum
  uint32_t src;       // source address
  uint32_t dst;       // dest address

  void swap_endian() {
    swap_endian_inplace(len);
    swap_endian_inplace(id);
    swap_endian_inplace(frag_off);
    swap_endian_inplace(sum);
    swap_endian_inplace(src);
    swap_endian_inplace(dst);
  }

  uint8_t hl() const { return get_bit_field<0, 4>(hl_ver); }
  uint8_t ver() const { return get_bit_field<4, 4>(hl_ver); }
  uint16_t off() const { return get_bit_field<0, 13>(frag_off); }
  uint16_t flags() const { return get_bit_field<13, 3>(frag_off); }
};
const int ip_header_size = sizeof(IPHeader);

// Big endian
#pragma pack(1)
struct UDPHeader {
  uint16_t sport;  // source port
  uint16_t dport;  // destination port
  uint16_t len;    // udp length
  uint16_t sum;    // udp checksum

  void swap_endian() {
    swap_endian_inplace(sport);
    swap_endian_inplace(dport);
    swap_endian_inplace(len);
    swap_endian_inplace(sum);
  }
};
const int udp_header_size = sizeof(UDPHeader);

#pragma pack(1)
struct RecordHeader {
  PCAPRecordHeader pcap;
  uint8_t dst_mac[6];
  uint8_t src_mac[6];
  uint16_t protocol_type;
  IPHeader ip;
  UDPHeader udp;

  void swap_endian() {
    ip.swap_endian();
    udp.swap_endian();
  }
};
const int record_header_size = sizeof(RecordHeader);
const int packet_header_size = record_header_size - pcap_record_header_size;

// -----------------------------------------------------------------------------
// Capture
// -----------------------------------------------------------------------------
SensorError Capture::open_for_read(const std::string &filename) {
  const auto error = open_for_read_impl(filename);
  if (error) close();
  return error;
}

SensorError Capture::open_for_read_impl(const std::string &filename) {
  close();

  m_filename = filename;
  m_is_read_mode = true;

  m_stream.open(filename.c_str(), std::ios::in | std::ios::binary);
  CHECK_FILE(m_stream)
  auto error = read_file_header();
  if (error) return error;

  bool index_loaded = false;
  const std::string index_path =
      filename + ".cep" + std::to_string(IndexFileHeader().version);
  {
    std::ifstream f(index_path.c_str(), std::ios::in | std::ios::binary);
    const auto error_tmp = load_read_index(f);
    if (!error_tmp) index_loaded = true;
  }
  if (!index_loaded) {
    error = build_read_index();
    if (error) return error;
    {
      std::ofstream f(index_path.c_str(),
                      std::ios::out | std::ios::trunc | std::ios::binary);
      save_read_index(f);
      f.close();
    }
  }
  if (!m_read_index.empty()) m_length = m_read_index.back().position;

  return CEPTON_SUCCESS;
}

SensorError Capture::open_for_write(const std::string &filename, bool append) {
  const auto error = open_for_write_impl(filename, append);
  if (error) close();
  return error;
}

SensorError Capture::open_for_write_impl(const std::string &filename,
                                         bool append) {
  close();

  m_filename = filename;
  m_is_read_mode = false;

  std::ios::openmode mode = std::ios::out | std::ios::binary;
  mode |= (append) ? std::ios::app : std::ios::trunc;
  m_stream.open(filename.c_str(), mode);
  CHECK_FILE(m_stream)
  if (!append) {
    auto error = write_file_header();
    if (error) return error;
  }
  return CEPTON_SUCCESS;
}

void Capture::close() {
  m_stream.close();
  m_filename = "";
  m_timestamp_offset = 0;
  m_start_time = 0;
  m_position = 0;
  m_length = 0;
  m_read_index.clear();
  m_packets.clear();
}

SensorError Capture::write_file_header() {
  m_stream.seekp(0);
  PCAPFileHeader file_header;
  file_header.magic = PCAP_MAGIC;
  file_header.version_major = PCAP_VERSION_MAJOR;
  file_header.version_minor = PCAP_VERSION_MINOR;
  file_header.thiszone = 0;
  file_header.sigfigs = 0;
  file_header.snaplen = 0xFFFF;
  file_header.linktype = PCAP_LINKTYPE_ETHERNET;
  write_value(m_stream, file_header);
  CHECK_FILE(m_stream)
  m_write_ptr = m_stream.tellp();
  return CEPTON_SUCCESS;
}

SensorError Capture::read_file_header() {
  m_stream.seekg(0);
  PCAPFileHeader file_header;
  read_value(m_stream, file_header);
  CHECK_FILE(m_stream)
  m_read_ptr = m_stream.tellg();
  if (file_header.magic != PCAP_MAGIC) return CEPTON_ERROR_INVALID_FILE_TYPE;
  m_position = 0;
  m_timestamp_offset = (int64_t)file_header.thiszone * util::second_usec;
  return CEPTON_SUCCESS;
}

SensorError read_record_header(std::istream &f, int64_t pointer,
                               RecordHeader &record_header) {
  f.seekg(pointer);
  if (is_eof(f)) return CEPTON_ERROR_EOF;
  read_value(f, record_header);
  CHECK_FILE(f)
  record_header.swap_endian();
  return CEPTON_SUCCESS;
}

SensorError Capture::build_read_index() {
  m_start_time = 0;
  m_read_index.clear();
  int64_t pointer = pcap_file_header_size;
  while (true) {
    RecordHeader record_header;
    auto error = read_record_header(m_stream, pointer, record_header);
    if (error) {
      if (error.code == CEPTON_ERROR_EOF) break;
      return error;
    }

    const int64_t timestamp = record_header.pcap.ts_sec * util::second_usec +
                              record_header.pcap.ts_usec + m_timestamp_offset;
    if (!m_start_time) m_start_time = timestamp;

    PacketIndex pi;
    pi.position = timestamp - m_start_time;
    pi.pointer = pointer;
    m_read_index.push_back(pi);

    if (record_header.pcap.incl_len == 0) return CEPTON_ERROR_CORRUPT_FILE;
    pointer += pcap_record_header_size + record_header.pcap.incl_len;
  }
  return CEPTON_SUCCESS;
}

SensorError Capture::load_read_index(std::ifstream &f) {
  m_read_index.clear();

  // Load header
  IndexFileHeader index_header;
  read_value(f, index_header);
  CHECK_FILE(f)
  if (index_header.version != IndexFileHeader().version)
    return CEPTON_ERROR_CORRUPT_FILE;
  if (index_header.n == 0) return CEPTON_ERROR_CORRUPT_FILE;
  m_start_time = index_header.start_time;

  // Load index
  m_read_index.resize(index_header.n);
  read_values(f, m_read_index.data(), index_header.n);
  CHECK_FILE(f)
  if (!is_eof(f)) return CEPTON_ERROR_CORRUPT_FILE;

  // Check start time
  {
    RecordHeader record_header;
    auto error = read_record_header(m_stream, m_read_index.front().pointer,
                                    record_header);
    if (error) return error;
    const int64_t timestamp = record_header.pcap.ts_sec * util::second_usec +
                              record_header.pcap.ts_usec + m_timestamp_offset;
    if (m_start_time != timestamp) return CEPTON_ERROR_CORRUPT_FILE;
  }

  // Check final position
  {
    RecordHeader record_header;
    auto error = read_record_header(m_stream, m_read_index.back().pointer,
                                    record_header);
    if (error) return error;
    const int64_t pointer = m_read_index.back().pointer +
                            pcap_record_header_size +
                            record_header.pcap.incl_len;
    m_stream.seekg(pointer);
    if (!is_eof(m_stream)) return CEPTON_ERROR_CORRUPT_FILE;
  }

  return CEPTON_SUCCESS;
}

SensorError Capture::save_read_index(std::ofstream &f) const {
  IndexFileHeader index_header;
  index_header.start_time = m_start_time;
  index_header.n = m_read_index.size();
  write_value(f, index_header);
  write_values(f, m_read_index.data(), index_header.n);
  CHECK_FILE(f)
  return CEPTON_SUCCESS;
}

SensorError Capture::seek(int64_t position) {
  if (!is_open()) return CEPTON_ERROR_NOT_OPEN;
  if ((position < 0) || (position >= m_length))
    return SensorError(CEPTON_ERROR_INVALID_ARGUMENTS,
                       "Invalid seek position!");

  PacketIndex pi;
  pi.position = position;
  auto iter = std::lower_bound(m_read_index.begin(), m_read_index.end(), pi,
                               [](const PacketIndex &a, const PacketIndex &b) {
                                 return a.position < b.position;
                               });
  if (iter == m_read_index.end()) return CEPTON_ERROR_EOF;
  m_position = iter->position;
  m_read_ptr = iter->pointer;
  return CEPTON_SUCCESS;
}

SensorError Capture::next_packet(PacketHeader &packet_header,
                                 const uint8_t *&packet_data) {
  while (true) {
    bool success;
    const auto error = next_packet_impl(success, packet_header, packet_data);
    if (error) return error;
    if (success) break;
  }

  return CEPTON_SUCCESS;
}

SensorError Capture::next_packet_impl(bool &success,
                                      PacketHeader &packet_header,
                                      const uint8_t *&packet_data) {
  success = false;

  RecordHeader record_header;
  auto error = read_record_header(m_stream, m_read_ptr, record_header);
  if (error) return error;
  m_read_ptr += pcap_record_header_size + record_header.pcap.incl_len;

  // Skip if invalid protocol
  if ((record_header.protocol_type != PROTOCOL_V4) ||
      (record_header.ip.p != IP_PROTOCOL_UDP))
    return CEPTON_SUCCESS;

  if (record_header.ip.hl_ver != IP_HL_VER) {
#ifdef CEPTON_INTERNAL
    api::log_error(SensorError(CEPTON_ERROR_CORRUPT_FILE, "Invalid IP header!"),
                   "Capture failed!");
#endif
    return CEPTON_SUCCESS;
  }

  const uint32_t ip_v4 = record_header.ip.src;
  auto &packet = m_packets[ip_v4];
  const int fragment_id = record_header.ip.id;
  const bool is_first_fragment = !record_header.ip.off();
  if (is_first_fragment) {
    packet.reset();
    packet.header.ip_v4 = ip_v4;
    packet.header.timestamp = record_header.pcap.ts_sec * util::second_usec +
                              record_header.pcap.ts_usec + m_timestamp_offset;
    packet.id = fragment_id;
    packet.len = record_header.udp.len;
    packet.buffer.resize(packet.len);
  } else {
    const int fragment_off =
        8 * (int)record_header.ip.off() - packet.n_fragments * udp_header_size;
    if ((packet.id != fragment_id) || (packet.off != fragment_off)) {
#ifdef CEPTON_INTERNAL
      api::log_error(
          SensorError(CEPTON_ERROR_CORRUPT_FILE, "Invalid fragment!"),
          "Capture failed!");
#endif
      return CEPTON_SUCCESS;
    }
  }
  ++packet.n_fragments;
  packet.mf = record_header.ip.flags() & IP_FLAG_MF;

  const int fragment_len = record_header.pcap.incl_len - packet_header_size;
  m_stream.read((char *)packet.buffer.data() + packet.off, fragment_len);
  CHECK_FILE(m_stream)
  packet.off += fragment_len;

  if (packet.mf) return CEPTON_SUCCESS;

  packet.len -= packet.n_fragments * udp_header_size;
  packet.buffer.resize(packet.len);
  if (packet.off != packet.len) {
#ifdef CEPTON_INTERNAL
    api::log_error(
        SensorError(CEPTON_ERROR_CORRUPT_FILE, "Invalid packet size!"),
        "Capture failed!");
#endif
    return CEPTON_SUCCESS;
  }
  packet.header.data_size = packet.buffer.size();

  if (m_start_time == 0) m_start_time = packet.header.timestamp;
  m_position = packet.header.timestamp - m_start_time;

  success = true;
  packet_header = packet.header;
  packet_data = packet.buffer.data();
  return CEPTON_SUCCESS;
}

SensorError Capture::append_packet(const Capture::PacketHeader &packet_header,
                                   const uint8_t *data) {
  if (!is_open_for_write()) return CEPTON_ERROR_NOT_OPEN;

  const int data_len = packet_header.data_size;

  RecordHeader record_header = {};
  record_header.protocol_type = PROTOCOL_V4;

  int64_t timestamp = (packet_header.timestamp) ? packet_header.timestamp
                                                : util::get_timestamp_usec();
  timestamp -= m_timestamp_offset;
  record_header.pcap.ts_sec = timestamp / util::second_usec;
  record_header.pcap.ts_usec = timestamp % util::second_usec;
  record_header.pcap.incl_len = packet_header_size + data_len;
  record_header.pcap.orig_len = packet_header_size + data_len;

  record_header.ip.hl_ver = IP_HL_VER;
  record_header.ip.len = ip_header_size + udp_header_size + data_len;
  record_header.ip.p = IP_PROTOCOL_UDP;
  record_header.ip.src = packet_header.ip_v4;
  record_header.ip.dst = 0xFFFFFFFF;

  record_header.udp.sport = 443;
  record_header.udp.dport = 8808;
  record_header.udp.len = udp_header_size + data_len;

  record_header.swap_endian();

  m_stream.seekp(m_write_ptr);
  write_value(m_stream, record_header);
  m_stream.write((const char *)data, data_len);
  CHECK_FILE(m_stream)
  m_write_ptr = m_stream.tellp();
  return CEPTON_SUCCESS;
}

}  // namespace cepton_sdk
