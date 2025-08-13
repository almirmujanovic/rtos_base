// udp_socket.cpp
#include "udp_socket.hpp"
#include <errno.h>
#include <sys/time.h>   // for struct timeval
#include <cstring>      // ✅ Add this for strerror()

using namespace espp;

// Small helper to set DSCP EF (46 -> TOS 0xB8) so Wi-Fi uses WMM video/voice queue.
static void set_dscp_ef(int sock, espp::Logger &log) {
  int tos = 0xB8; // DSCP EF (46) << 2
  if (setsockopt(sock, IPPROTO_IP, IP_TOS, &tos, sizeof(tos)) < 0) {
    log.warn("UdpSocket: IP_TOS(DSCP EF) failed: {}", errno);
  }
}

// Apply common UDP socket QoS/buffer/timeouts (works for both sender and receiver sockets)
static void tune_udp_socket(int sock, espp::Logger &log) {
  // Priority (WMM) – do this first
  set_dscp_ef(sock, log);

  // Larger lwIP socket buffers (advisory but helps)
  int sndbuf = 128 * 1024;  // 128 KB
  if (setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0) {
    log.warn("UdpSocket: SO_SNDBUF failed: {}", errno);
  }
  int rcvbuf = 64 * 1024;   // 64 KB
  if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
    log.warn("UdpSocket: SO_RCVBUF failed: {}", errno);
  }

  // Short send/recv timeouts so we never block long; we handle backoff ourselves.
  struct timeval tv;
  tv.tv_sec = 0; tv.tv_usec = 15000; // 15 ms
  if (setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
    log.warn("UdpSocket: SO_SNDTIMEO failed: {}", errno);
  }
  if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    log.warn("UdpSocket: SO_RCVTIMEO failed: {}",  errno);
  }
}

UdpSocket::UdpSocket(const UdpSocket::Config &config)
    : Socket(Type::DGRAM, Logger::Config{.tag = "UdpSocket", .level = config.log_level}) {
  // IMPORTANT: tune the UDP socket used by the RTP sender too (this ctor is used in both paths)
  if (is_valid()) {
    tune_udp_socket(socket_, logger_);
  }
}

UdpSocket::~UdpSocket() {
  cleanup();
}

bool UdpSocket::send(const std::vector<uint8_t> &data, const UdpSocket::SendConfig &send_config) {
  return send(std::string_view{(const char *)data.data(), data.size()}, send_config);
}

bool UdpSocket::send(std::string_view data, const UdpSocket::SendConfig &send_config) {
  if (!is_valid()) {
    logger_.error("Socket invalid, cannot send");
    return false;
  }
  if (send_config.is_multicast_endpoint) {
    if (!make_multicast()) {
      logger_.error("Cannot make multicast: {}", error_string());
      return false;
    }
  }
  if (send_config.wait_for_response) {
    if (!set_receive_timeout(send_config.response_timeout)) {
      logger_.error("Could not set receive timeout to {}: {}",
                    send_config.response_timeout.count(), error_string());
      return false;
    }
  }

  Socket::Info server_info;
  server_info.init_ipv4(send_config.ip_address, send_config.port);
  auto server_address = server_info.ipv4_ptr();

  // DEBUG instead of INFO to avoid log-induced jitter on every RTP packet
  logger_.debug("Client sending {} bytes to {}:{}", data.size(),
                send_config.ip_address, send_config.port);

  // Congestion-aware send: brief retries on ENOMEM/ENOBUFS/EAGAIN
  constexpr int kMaxRetries = 3;
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    int num_bytes_sent = sendto(socket_, data.data(), data.size(), 0,
                                (struct sockaddr *)server_address, sizeof(*server_address));
    if (num_bytes_sent >= 0) {
      logger_.debug("Client sent {} bytes", num_bytes_sent);
      break;
    }

    // grab errno once (lwIP maps ERR_MEM to ENOMEM)
    int err = errno;
    if (err == ENOMEM || err == ENOBUFS || err == EAGAIN) {
      // let lwIP/NIC drain a bit; keep it tiny so we don't add latency
      vTaskDelay(pdMS_TO_TICKS(1));
      if (attempt + 1 < kMaxRetries) continue; // one more try
    }

    // hard failure or out of retries
    logger_.error("Error occurred during sending: {}", error_string());
    return false;
  }

  if (!send_config.wait_for_response) return true;

  if (send_config.response_size == 0) {
    logger_.warn("Response requested, but response_size=0, not waiting for response!");
    return true;
  }

  std::vector<uint8_t> received_data;
  logger_.info("Client waiting for response");
  if (!receive(send_config.response_size, received_data, server_info)) {
    logger_.warn("Client could not get response");
    return false;
  }
  logger_.info("Client got {} bytes of response", received_data.size());
  if (send_config.on_response_callback) {
    logger_.debug("Client calling response callback");
    send_config.on_response_callback(received_data);
  }
  return true;
}

bool UdpSocket::receive(size_t max_num_bytes, std::vector<uint8_t> &data,
                        Socket::Info &remote_info) {
  if (!is_valid()) {
    logger_.error("Socket invalid, cannot receive.");
    return false;
  }
  auto remote_address = remote_info.ipv4_ptr();
  socklen_t socklen = sizeof(*remote_address);

  std::unique_ptr<uint8_t[]> receive_buffer(new uint8_t[max_num_bytes]());
  logger_.debug("Receiving up to {} bytes", max_num_bytes);

  int num_bytes_received = recvfrom(socket_, (char *)receive_buffer.get(), max_num_bytes, 0,
                                    (struct sockaddr *)remote_address, &socklen);
  if (num_bytes_received < 0) {
    logger_.debug("Receive failed: {}", error_string());
    return false;
  }
  uint8_t *data_ptr = (uint8_t *)receive_buffer.get();
  data.assign(data_ptr, data_ptr + num_bytes_received);
  remote_info.update();
  logger_.debug("Received {} bytes from {}", num_bytes_received, remote_info);
  return true;
}

bool UdpSocket::start_receiving(Task::BaseConfig &task_config,
                                const UdpSocket::ReceiveConfig &receive_config) {
  if (task_ && task_->is_started()) {
    logger_.error("Server is alrady receiving");
    return false;
  }
  if (!is_valid()) {
    logger_.error("Socket invalid, cannot start receiving.");
    return false;
  }

  // tune this bound socket too
  tune_udp_socket(socket_, logger_);

  server_receive_callback_ = receive_config.on_receive_callback;

  struct sockaddr_in server_addr;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_family = address_family_;
  server_addr.sin_port = htons(receive_config.port);
  int err = bind(socket_, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (err < 0) {
    logger_.error("Unable to bind: {}", error_string());
    return false;
  }

  if (receive_config.is_multicast_endpoint) {
    if (!make_multicast()) {
      logger_.error("Unable to make bound socket multicast: {}", error_string());
      return false;
    }
    if (!add_multicast_group(receive_config.multicast_group)) {
      logger_.error("Unable to add multicast group to bound socket: {}", error_string());
      return false;
    }
  }

  using namespace std::placeholders;
  task_ = Task::make_unique({
      .callback = std::bind(&UdpSocket::server_task_function, this,
                            receive_config.buffer_size, _1, _2, _3),
      .task_config = task_config,
  });
  task_->start();
  return true;
}
