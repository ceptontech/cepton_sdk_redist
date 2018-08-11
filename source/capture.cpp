#include "cepton_sdk/capture.hpp"

#include <cstring>

#include <algorithm>
#include <fstream>
#include <string>

#include "cepton_sdk_util.hpp"

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

// Internal pcap file formats
#pragma pack(push, 1)
struct PCAPFileHeader {
  uint32_t magic;
  uint16_t version_major;
  uint16_t version_minor;
  uint32_t thiszone; /* gmt to local correction */
  uint32_t sigfigs;  /* accuracy of timestamps */
  uint32_t snaplen;  /* max length saved portion of each packet */
  uint32_t linktype; /* data link type (LINKTYPE_*) */
};

struct PCAPRecordHeader {
  uint32_t ts_sec;
  uint32_t ts_usec;
  uint32_t incl_len;
  uint32_t orig_len;
};

struct IPHeader {
  uint8_t ip_hl_v;  // header length + version
  uint8_t ip_tos;   // type of service
  uint16_t ip_len;  // total length

  uint16_t ip_id;   // identification
  uint16_t ip_off;  // fragment offset field

  uint8_t ip_ttl;   // time to live
  uint8_t ip_p;     // protocol
  uint16_t ip_sum;  // checksum

  uint32_t ip_src, ip_dst;  // source and dest address
};

struct UDPHeader {
  uint8_t dst_mac[6];
  uint8_t src_mac[6];
  uint16_t protocol_type;
  IPHeader ip;

  uint16_t uh_sport;  // source port (BIG ENDIAN)
  uint16_t uh_dport;  // destination port (BIG ENDIAN)
  uint16_t uh_ulen;   // udp length (BIG ENDIAN)
  uint16_t uh_sum;    // udp checksum
};

// For ease of writing to files
struct RecordHeader {
  PCAPRecordHeader pcap;
  UDPHeader udp;
};
#pragma pack(pop)

const uint8_t default_udp_header_data[sizeof(UDPHeader)] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x18, 0x12, 0x12, 0,    0,
    0,    0x08, 0x00, 0x45, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x80, 0x11, 0,    0,    0,    0,    0,    0,    0xFF, 0xFF, 0xFF,
    0xFF, 0x01, 0xBB, 0x09, 0x40, 0x00, 0x00, 0x00, 0x00};
const UDPHeader default_udp_header =
    *(const UDPHeader *)default_udp_header_data;

// Constants (copied from various sources)
const uint32_t LINKTYPE_ETHERNET = 1;
const uint32_t PCAP_MAGIC = 0xA1B2C3D4;  // low endian
const uint16_t PCAP_MAJOR = 2;
const uint16_t PCAP_MINOR = 4;

const uint16_t PROTOCOL_V4 = 0x0008;  // NOTICE this is BIG ENDIAN
const uint8_t IP_PROTO_UDP = 0x11;
// const uint8_t IPVER_BYTE = 0x45;  // IPv4 + 20 bytes header

uint16_t swap_uint16(uint16_t val) { return (val >> 8) | (val << 8); }

uint32_t swap_uint32(uint32_t val) {
  val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
  return (val << 16) | (val >> 16);
}

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
  m_buffer.clear();
  m_timestamp_offset = 0;
  m_start_time = 0;
  m_position = 0;
  m_length = 0;
  m_read_index.clear();
}

SensorError Capture::write_file_header() {
  m_stream.seekp(0);
  PCAPFileHeader file_header;
  file_header.magic = PCAP_MAGIC;
  file_header.version_major = PCAP_MAJOR;
  file_header.version_minor = PCAP_MINOR;
  file_header.thiszone = 0;
  file_header.sigfigs = 0;
  file_header.snaplen = 0xFFFF;
  file_header.linktype = LINKTYPE_ETHERNET;
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
  m_timestamp_offset = file_header.thiszone;
  return CEPTON_SUCCESS;
}

SensorError read_record_header(std::istream &f, int64_t pointer,
                               RecordHeader &record_header) {
  f.seekg(pointer);
  if (is_eof(f)) return CEPTON_ERROR_EOF;
  read_value(f, record_header);
  CHECK_FILE(f)
  return CEPTON_SUCCESS;
}

SensorError Capture::build_read_index() {
  m_start_time = 0;
  m_read_index.clear();
  int64_t pointer = sizeof(PCAPFileHeader);
  while (true) {
    RecordHeader record_header;
    auto error = read_record_header(m_stream, pointer, record_header);
    if (error) {
      if (error.code == CEPTON_ERROR_EOF) break;
      return error;
    }

    const int64_t timestamp = record_header.pcap.ts_sec * util::second_usec +
                              record_header.pcap.ts_usec;
    if (!m_start_time) m_start_time = timestamp;

    PacketIndex pi;
    pi.position = timestamp - m_start_time;
    pi.pointer = pointer;
    m_read_index.push_back(pi);

    if (record_header.pcap.incl_len == 0) return CEPTON_ERROR_CORRUPT_FILE;
    pointer += sizeof(PCAPRecordHeader) + record_header.pcap.incl_len;
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
                              record_header.pcap.ts_usec;
    if (m_start_time != timestamp) return CEPTON_ERROR_CORRUPT_FILE;
  }

  // Check final position
  {
    RecordHeader record_header;
    auto error = read_record_header(m_stream, m_read_index.back().pointer,
                                    record_header);
    if (error) return error;
    const int64_t pointer = m_read_index.back().pointer +
                            sizeof(PCAPRecordHeader) +
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

  // Skip if invalid protocol
  if ((record_header.udp.protocol_type != PROTOCOL_V4) ||
      (record_header.udp.ip.ip_p != IP_PROTO_UDP)) {
    m_read_ptr += sizeof(PCAPRecordHeader) + record_header.pcap.incl_len;
    return CEPTON_SUCCESS;
  }

  int len = swap_uint16(record_header.udp.uh_ulen);
  if (record_header.pcap.incl_len <= sizeof(UDPHeader))
    return SensorError(CEPTON_ERROR_CORRUPT_FILE, "Invalid record size!");

  // Sanity check
  // 8 is the UDP header size
  if ((record_header.pcap.incl_len - sizeof(UDPHeader)) != (len - 8))
    return SensorError(CEPTON_ERROR_CORRUPT_FILE, "Invalid record size!");
  len -= 8;

  packet_header.ip_v4 = swap_uint32(record_header.udp.ip.ip_src);
  packet_header.timestamp = record_header.pcap.ts_sec * util::second_usec +
                            record_header.pcap.ts_usec;
  packet_header.data_size = len;

  if (m_start_time == 0) m_start_time = packet_header.timestamp;
  m_position = packet_header.timestamp - m_start_time;

  m_buffer.resize(len);
  m_stream.read((char *)m_buffer.data(), len);
  CHECK_FILE(m_stream)
  m_read_ptr = m_stream.tellg();

  success = true;
  packet_data = m_buffer.data();
  return CEPTON_SUCCESS;
}

SensorError Capture::append_packet(const Capture::PacketHeader &packet_header,
                                   const uint8_t *data) {
  if (!is_open_for_write()) return CEPTON_ERROR_NOT_OPEN;

  const int data_len = packet_header.data_size;

  RecordHeader record_header;
  if (packet_header.timestamp) {
    record_header.pcap.ts_sec = packet_header.timestamp / util::second_usec;
    record_header.pcap.ts_usec = packet_header.timestamp % util::second_usec;
  } else {
    const auto timestamp = util::get_timestamp_usec();
    record_header.pcap.ts_sec = timestamp / util::second_usec;
    record_header.pcap.ts_usec = timestamp % util::second_usec;
  }
  record_header.pcap.incl_len = data_len + sizeof(UDPHeader);
  record_header.pcap.orig_len = data_len + sizeof(UDPHeader);
  record_header.udp = default_udp_header;
  record_header.udp.ip.ip_src = swap_uint32(packet_header.ip_v4);
  record_header.udp.ip.ip_len = swap_uint16(data_len + 28);
  record_header.udp.uh_ulen = swap_uint16(data_len + 8);

  m_stream.seekp(m_write_ptr);
  write_value(m_stream, record_header);
  m_stream.write((const char *)data, data_len);
  CHECK_FILE(m_stream)
  m_write_ptr = m_stream.tellp();
  return CEPTON_SUCCESS;
}

}  // namespace cepton_sdk
