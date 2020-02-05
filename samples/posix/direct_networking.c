#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <time.h>

#include <cepton_sdk.h>

char global_buffer[2000];

/* Networking code */

int create_socket() {
  int socket_desc = socket(AF_INET, SOCK_DGRAM, 0);

  if (socket_desc == -1) {
    printf("Could not create UDP socket\n");
    exit(-1);
  }

  return socket_desc;
}

int connect_to_sensor() {
  int socket_desc;
  struct sockaddr_in sensor_addr;

  sensor_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  sensor_addr.sin_family = AF_INET;
  sensor_addr.sin_port = htons(8808);

  socket_desc = create_socket();

  if (bind(socket_desc, (struct sockaddr *)&sensor_addr, sizeof(sensor_addr)) < 0) {
    printf("Bind error\n");
    exit(-1);
  }

  return socket_desc;
}

int receive_udp_data(int socket_desc, unsigned long *from) {
  int received;
  struct sockaddr_in from_addr;
  socklen_t addrlen = sizeof(from_addr);
  received = recvfrom(socket_desc, global_buffer, sizeof(global_buffer), 0,
    (struct sockaddr *)&from_addr, &addrlen);
  if (received < 0) {
    printf("Recv failed\n");
    return 0;
  }
  *from = htonl(from_addr.sin_addr.s_addr);
  return received;
}

/* Get high precision time */
long long get_hp_time() {
  struct timespec time;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time);
  return time.tv_sec * 1000000 + (time.tv_nsec + 500) / 1000; // convert to microseconds
}

/* Cepton SDK callbacks */

void check_sdk_error() {
  const char *error_msg;
  int error_code = cepton_sdk_get_error(&error_msg);
  if (error_code != CEPTON_SUCCESS) {
    printf("%s: %s\n", cepton_get_error_code_name(error_code), error_msg);
    exit(1);
  }
}

int n_frames = 0;

void image_frame_callback(CeptonSensorHandle handle, size_t n_points,
                          const struct CeptonSensorImagePoint *c_points,
                          void *user_data) {
  ++n_frames;
  printf("Got %d frames\n", n_frames);
}

void cepton_sdk_error_handler(CeptonSensorHandle handle,
                              CeptonSensorErrorCode error_code,
                              const char *error_msg, const void *error_data,
                              size_t error_data_size, void *user_data) {
  printf("Got error: %s\n", error_msg);
}


int main(){
  int received;
  int socket_desc = connect_to_sensor();

  // Initialize cepton SDK
  struct CeptonSDKOptions options = cepton_sdk_create_options();
  options.frame.mode = CEPTON_SDK_FRAME_COVER;
  options.control_flags |= CEPTON_SDK_CONTROL_DISABLE_NETWORK;
  cepton_sdk_initialize(CEPTON_SDK_VERSION, &options, cepton_sdk_error_handler, NULL);
  check_sdk_error();

  cepton_sdk_listen_image_frames(image_frame_callback, NULL);
  check_sdk_error();

  while(n_frames < 10) {
    unsigned long handle;
    received = receive_udp_data(socket_desc, &handle);
    if (received <= 0) break;
    cepton_sdk_mock_network_receive(handle, get_hp_time(), (const uint8_t *)global_buffer, received);
    check_sdk_error();
  }

  cepton_sdk_deinitialize();

  return 0;
}
