#include "cepton_sdk/network.hpp"

#include "cepton_sdk/core.hpp"
#include "cepton_sdk/sensor.hpp"

namespace cepton_sdk {

NetworkManager network_manager;

NetworkSocket::NetworkSocket(asio::io_service &io, uint16_t port)
    : asio::ip::udp::socket(io) {
  asio::error_code error;
  open(asio::ip::udp::v4());
  set_option(asio::socket_base::reuse_address(true));
  bind(asio::ip::udp::endpoint(asio::ip::address_v4::any(), port), error);
  if (error) abort();

  listen();
}

void NetworkSocket::listen() {
  async_receive_from(
      asio::buffer(sensor_socket_buffer), end_point,
      [this](const asio::error_code &error, std::size_t bytes_transferred) {
        handle_receive(error, bytes_transferred);
      });
}

void NetworkSocket::handle_receive(const asio::error_code &error,
                                   std::size_t bytes_transferred) {
  if (error == asio::error::operation_aborted) return;

  uint64_t addr = end_point.address().to_v4().to_ulong();
  if (error.value()) {
    m_last_error =
        error.value();  // This value will not survive the emit_error call
    callback_manager.emit_error(CEPTON_NULL_HANDLE, CEPTON_ERROR_COMMUNICATION,
                                error.message().c_str(), &m_last_error);
    return;
  }
  callback_manager.network_cb.emit(addr, sensor_socket_buffer,
                                   bytes_transferred);
  sensor_manager.handle_network_receive(addr, sensor_socket_buffer,
                                        bytes_transferred);

  listen();
}

void NetworkManager::initialize() {
  deinitialize();
  if (sdk_manager.has_control_flag(CEPTON_SDK_CONTROL_DISABLE_NETWORK)) return;

  std::lock_guard<std::mutex> lock(m_mutex);

  m_io_service_ptr.reset(new asio::io_service());

  m_sensor_socket_ptr =
      std::make_shared<NetworkSocket>(*m_io_service_ptr, m_port);

  m_io_service_thread_ptr.reset(new std::thread([this]() {
    std::shared_ptr<asio::io_service> local_io_service_ptr;
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      if (!m_io_service_ptr) return;
      local_io_service_ptr = m_io_service_ptr;
    }
    try {
      local_io_service_ptr->run();
    } catch (const std::exception &) {
      // std::cerr << e.what() << "\n";
    }
  }));
}

void NetworkManager::deinitialize() {
  std::lock_guard<std::mutex> lock(m_mutex);

  if (m_sensor_socket_ptr) {
    try {
      m_sensor_socket_ptr->shutdown(asio::ip::udp::socket::shutdown_both);
      m_sensor_socket_ptr->close();
    } catch (std::system_error) {
      // On OSX we will get system_error: shutdown: Socket is not connected
      // This is safe to ignore
    }
  }

  if (m_io_service_ptr) {
    m_io_service_ptr->stop();
  }

  if (m_io_service_thread_ptr) {
    m_io_service_thread_ptr->join();
    m_io_service_thread_ptr.reset();
  }
  // Don't delete socket until join() is successful
  if (m_sensor_socket_ptr) m_sensor_socket_ptr.reset();

  if (m_io_service_ptr) {
    m_io_service_ptr.reset();
  }
}

uint16_t NetworkManager::get_port() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_port;
}

CeptonSensorErrorCode NetworkManager::set_port(uint16_t port) {
  deinitialize();
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_port = port;
  }
  if (sdk_manager.is_initialized()) initialize();
  return CEPTON_SUCCESS;
}

}  // namespace cepton_sdk
