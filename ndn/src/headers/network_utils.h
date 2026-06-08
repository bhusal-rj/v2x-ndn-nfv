#ifndef NETWORK_UTILS_H
#define NETWORK_UTILS_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <string>

int create_socket(std::string host, int port);
ssize_t send_data(int sockfd, const std::string& data);
ssize_t receive_data(int sockfd, char* buffer, size_t buffer_size);

#endif