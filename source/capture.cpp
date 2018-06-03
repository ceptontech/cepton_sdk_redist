#include "cepton_sdk/capture.hpp"

#include <cstring>

#include <algorithm>
#include <fstream>
#include <string>

#include "cepton_sdk_util.hpp"

#include <iostream>

namespace cepton_sdk {

// Internal pcap file formats
#pragma pack(push, 1)
struct pcap_file_header {
  uint32_t magic;
  uint16_t version_major;
  uint16_t version_minor;
  uint32_t thiszone; /* gmt to local correction */
  uint32_t sigfigs;  /* accuracy of timestamps */
  uint32_t snaplen;  /* max length saved portion of each pkt */
  uint32_t linktype; /* data link type (LINKTYPE_*) */
};

struct pcap_rec_header {
  uint32_t ts_sec;
  uint32_t ts_usec;
  uint32_t incl_len;
  uint32_t orig_len;
};

// netinet/ip.h
struct ip {
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

struct eth_ip_udp_header {
  // eth frame
  uint8_t dst_mac[6];
  uint8_t src_mac[6];
  uint16_t protocol_type;
  // ip header
  struct ip ip_hdr;

  // udp header
  uint16_t uh_sport;  // source port (BIG ENDIAN)
  uint16_t uh_dport;  // destination port (BIG ENDIAN)
  uint16_t uh_ulen;   // udp length (BIG ENDIAN)
  uint16_t uh_sum;    // udp checksum
};

// For ease of writing to files
struct aggregated_header {
  pcap_rec_header rec_hdr;
  eth_ip_udp_header udp_hdr;
};
#pragma pack(pop)

const uint8_t default_eth_ip_udp_header[sizeof(eth_ip_udp_header)] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x18, 0x12, 0x12, 0,    0,
    0,    0x08, 0x00, 0x45, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x80, 0x11, 0,    0,    0,    0,    0,    0,    0xFF, 0xFF, 0xFF,
    0xFF, 0x01, 0xBB, 0x09, 0x40, 0x00, 0x00, 0x00, 0x00};

// Constants (copied from various sources)
const uint32_t LINKTYPE_ETHERNET = 1;
const uint32_t PCAP_MAGIC = 0xA1B2C3D4;  // low endian
const uint16_t PCAP_MAJOR = 2;
const uint16_t PCAP_MINOR = 4;

const uint16_t PROTOCOL_V4 = 0x0008;  // NOTICE this is BIG ENDIAN
const uint8_t IP_PROTO_UDP = 0x11;
// const uint8_t IPVER_BYTE = 0x45;  // IPv4 + 20 bytes header

inline uint32_t swap_uint32(uint32_t val) {
  val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
  return (val << 16) | (val >> 16);
}

inline uint16_t swap_uint16(uint16_t val) { return (val >> 8) | (val << 8); }

Capture::~Capture() { close(); }

bool Capture::open_for_read(std::string const &fname) {
  close();
  m_fh = fopen(fname.c_str(), "rb");
  m_filename = fname;
  if (!m_fh) {
    close();
    m_error_code = CEPTON_ERROR_FILE_IO;
    return false;
  }
  if (!read_file_header()) {
    close();
    m_error_code = CEPTON_ERROR_FILE_IO;
    return false;
  }
  m_is_read_mode = true;

  bool index_loaded = false;

  const std::string index_path =
      fname + ".cep" + std::to_string(IndexFileHeader().version);
  {
    std::ifstream f(index_path.c_str(), std::ios::in | std::ios::binary);
    if (f) index_loaded = load_read_index(f);
  }
  if (!index_loaded) {
    build_read_index();
    {
      std::ofstream f(index_path.c_str(),
                      std::ios::out | std::ios::trunc | std::ios::binary);
      if (f) save_read_index(f);
    }
  }
  if (!m_read_index.empty()) m_total_usec = m_read_index.back().time_usec;
  return true;
}

bool Capture::open_for_write(std::string const &fname, bool append) {
  close();
  m_fh = fopen(fname.c_str(), append ? "a+b" : "w+b");
  m_filename = fname;
  if (!m_fh) {
    close();
    m_error_code = CEPTON_ERROR_FILE_IO;
    return false;
  }
  if (append) {
    fseek(m_fh, 0, SEEK_END);
  } else {
    write_file_header();
  }
  m_is_read_mode = false;
  return true;
}

void Capture::close() {
  if (m_fh) {
    fclose(m_fh);
    m_fh = nullptr;
  }
  m_filename = "";
  m_read_usec = 0;
  m_total_usec = 0;
  m_start_time = 0;
  m_timestamp_offset = 0;
  m_buffer.clear();
  m_read_index.clear();
}

void Capture::write_file_header() {
  fseek(m_fh, 0, SEEK_SET);  // Go to the beginning

  pcap_file_header hdr;
  hdr.magic = PCAP_MAGIC;
  hdr.version_major = PCAP_MAJOR;
  hdr.version_minor = PCAP_MINOR;
  hdr.thiszone = 0;
  hdr.sigfigs = 0;
  hdr.snaplen = 0xFFFF;
  hdr.linktype = LINKTYPE_ETHERNET;
  fwrite(&hdr, sizeof(hdr), 1, m_fh);
  m_write_pointer = ftell(m_fh);
}

bool Capture::read_file_header() {
  fseek(m_fh, 0, SEEK_SET);
  pcap_file_header hdr;
  if (fread(&hdr, sizeof(hdr), 1, m_fh) != 1) {
    m_error_code = CEPTON_ERROR_INVALID_FILE_TYPE;
    return false;
  }
  if (hdr.magic != PCAP_MAGIC) {
    m_error_code = CEPTON_ERROR_INVALID_FILE_TYPE;
    return false;
  }

  m_read_pointer = sizeof(hdr);
  m_read_usec = 0;
  m_timestamp_offset = hdr.thiszone;

  return true;
}

void Capture::build_read_index() {
  m_read_index.clear();
  PacketIndex pi;
  pcap_rec_header rhdr;
  long file_ptr = sizeof(pcap_file_header);
  for (;;) {
    if (fseek(m_fh, file_ptr, SEEK_SET)) break;           // Failed to seek
    if (fread(&rhdr, sizeof(rhdr), 1, m_fh) != 1) break;  // Done with file
    if (m_start_time == 0)
      m_start_time = rhdr.ts_sec * 1000000LL + rhdr.ts_usec;
    pi.time_usec = rhdr.ts_sec * 1000000LL + rhdr.ts_usec - m_start_time;
    pi.read_ptr = file_ptr;
    m_read_index.push_back(pi);  // Copy in
    file_ptr += sizeof(pcap_rec_header) + rhdr.incl_len;
    if (rhdr.incl_len == 0) break;  // Sanity check
  }
}

template <typename T>
bool read_value(std::ifstream &f, T &value, int n = 1) {
  f.read(reinterpret_cast<char *>(&value), n * sizeof(T));
  return bool(f);
}

bool Capture::load_read_index(std::ifstream &f) {
  m_read_index.clear();

  IndexFileHeader hdr;
  if (!read_value(f, hdr)) return false;
  if (hdr.version != IndexFileHeader().version) return false;
  m_start_time = hdr.start_time;
  m_read_index.resize(hdr.n);
  if (!read_value(f, m_read_index[0], hdr.n)) return false;
  f.peek();
  return f.eof();
}

template <typename T>
void write_value(std::ofstream &f, const T &value, int n = 1) {
  f.write(reinterpret_cast<const char *>(&value), n * sizeof(T));
}

void Capture::save_read_index(std::ofstream &f) const {
  IndexFileHeader hdr;
  hdr.start_time = m_start_time;
  hdr.n = m_read_index.size();
  write_value(f, hdr);
  write_value(f, m_read_index[0], hdr.n);
  f.close();
}

bool Capture::seek(int64_t usec) {
  if (!is_open()) {
    m_error_code = CEPTON_ERROR_NOT_OPEN;
    return false;
  }

  if ((usec < 0) || (usec >= m_total_usec)) {
    m_error_code = CEPTON_ERROR_EOF;
    return false;
  }

  PacketIndex pi;
  pi.time_usec = usec;
  auto iter = std::lower_bound(m_read_index.begin(), m_read_index.end(), pi,
                               [](const PacketIndex &a, const PacketIndex &b) {
                                 return a.time_usec < b.time_usec;
                               });
  if (iter == m_read_index.end()) {
    m_error_code = CEPTON_ERROR_EOF;
    return false;
  }

  PacketIndex &ppi = *iter;
  m_read_pointer = ppi.read_ptr;
  m_read_usec = ppi.time_usec;
  return true;
}

CeptonSensorErrorCode Capture::next_packet(const PacketHeader **pkt_header,
                                           const uint8_t **pkt_data) {
  if (!is_open_for_read()) {
    m_error_code = CEPTON_ERROR_NOT_OPEN;
    return 0;
  }

  aggregated_header hdr;
// Always seek in case last read was unsuccessful and leave the pointer
// somewhere in the middle
l_try_again:
  fseek(m_fh, (long)m_read_pointer, SEEK_SET);
  size_t hdrlen = fread(&hdr, sizeof(hdr), 1, m_fh);

  // Failed to read
  if (hdrlen == 0) {
    m_error_code = CEPTON_ERROR_FILE_IO;
    return 0;
  }

  // Skip if protocol is not IPV4
  if (hdr.udp_hdr.protocol_type != PROTOCOL_V4) {
    m_read_pointer += sizeof(pcap_rec_header) + hdr.rec_hdr.incl_len;
    goto l_try_again;  // Clearer to do goto than a weird loop
  }

  // Skip if IP protocol is not UDP
  if (hdr.udp_hdr.ip_hdr.ip_p != IP_PROTO_UDP) {
    m_read_pointer += sizeof(pcap_rec_header) + hdr.rec_hdr.incl_len;
    goto l_try_again;  // Clearer to do goto than a weird loop
  }

  // Skip if port is not the right one (big endian)
  // 8808 = 0x2268 => 0x6822
  // 2368 = 0x0940 => 0x4009
  // if ((hdr.udp_hdr.uh_dport != 0x6822) && (hdr.udp_hdr.uh_dport != 0x4009)) {
  //  m_read_pointer += sizeof(pcap_rec_header) + hdr.rec_hdr.incl_len;
  //  goto l_try_again;
  //}

  unsigned len = swap_uint16(hdr.udp_hdr.uh_ulen);
  if (hdr.rec_hdr.incl_len <= sizeof(eth_ip_udp_header)) {
    m_error_code = CEPTON_ERROR_CORRUPT_FILE;
    return 0;  // len includes this
  }

  // Sanity check
  // 8 is the UDP header size
  if (hdr.rec_hdr.incl_len - sizeof(eth_ip_udp_header) != len - 8) {
    m_error_code = CEPTON_ERROR_CORRUPT_FILE;
    return 0;
  }
  len -= 8;

  m_temp_header.ip_v4 = swap_uint32(hdr.udp_hdr.ip_hdr.ip_src);
  if (hdr.udp_hdr.src_mac[0] != 0x18 || hdr.udp_hdr.src_mac[1] != 0x12 ||
      hdr.udp_hdr.src_mac[2] != 0x12) {
    m_temp_header.serial_number = 0;  // Invalid header 3 bytes for MAC
  } else {
    m_temp_header.serial_number = (hdr.udp_hdr.src_mac[3] << 16) +
                                  (hdr.udp_hdr.src_mac[4] << 8) +
                                  hdr.udp_hdr.src_mac[5];
  }
  m_temp_header.tv_usec = hdr.rec_hdr.ts_sec * 1000000LL + hdr.rec_hdr.ts_usec;
  m_temp_header.data_size = len;

  if (m_start_time == 0) {
    m_start_time = m_temp_header.tv_usec;
  }
  m_read_usec = m_temp_header.tv_usec - m_start_time;

  m_buffer.resize(len);
  size_t readlen = fread(m_buffer.data(), len, 1, m_fh);
  if (readlen < 1) {
    m_error_code = CEPTON_ERROR_CORRUPT_FILE;
    return 0;  // Failed to get the whole packet
  }

  m_read_pointer = ftell(m_fh);

  *pkt_header = &m_temp_header;
  *pkt_data = m_buffer.data();
  return len;
}

bool Capture::append_packet(const Capture::PacketHeader *hdr,
                            const uint8_t *data) {
  if (!is_open_for_write()) {
    m_error_code = CEPTON_ERROR_NOT_OPEN;
    return false;
  }

  aggregated_header h;
  std::size_t data_len = hdr->data_size;
  h.rec_hdr.ts_sec = (uint32_t)(hdr->tv_usec / 1000000LL);
  h.rec_hdr.ts_usec = (uint32_t)(hdr->tv_usec % 1000000LL);
  h.rec_hdr.incl_len = (uint32_t)(data_len + sizeof(eth_ip_udp_header));
  h.rec_hdr.orig_len = (uint32_t)(data_len + sizeof(eth_ip_udp_header));
  static_assert(sizeof(default_eth_ip_udp_header) == sizeof(h.udp_hdr),
                "Fix default package");
  std::memcpy(&h.udp_hdr, default_eth_ip_udp_header,
              sizeof(default_eth_ip_udp_header));
  uint8_t *pSerial = (uint8_t *)&hdr->serial_number;
  h.udp_hdr.src_mac[3] = pSerial[2];
  h.udp_hdr.src_mac[4] = pSerial[1];
  h.udp_hdr.src_mac[5] = pSerial[0];
  h.udp_hdr.ip_hdr.ip_src = swap_uint32(hdr->ip_v4);
  h.udp_hdr.ip_hdr.ip_len = swap_uint16((uint16_t)(data_len + 28));
  h.udp_hdr.uh_ulen = swap_uint16((uint16_t)(data_len + 8));

  if (hdr->tv_usec == 0) {
    auto timestamp_usec = get_timestamp_usec();
    h.rec_hdr.ts_sec = (uint32_t)(timestamp_usec / 1000000LL);
    h.rec_hdr.ts_usec = (uint32_t)(timestamp_usec % 1000000LL);
  }

  fseek(m_fh, (long)m_write_pointer, SEEK_SET);
  fwrite(&h, sizeof(h), 1, m_fh);
  fwrite(data, data_len, 1, m_fh);

  m_write_pointer = ftell(m_fh);

  return true;
}

}  // namespace cepton_sdk
