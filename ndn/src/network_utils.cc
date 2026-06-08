#include "network_utils.h"
#include <iostream>
#include <arpa/inet.h>
#include <cstring>


// Create the socket and connect to the server specified by the HOST and the PORT
// If the connection is successful, return the socket file descriptor.
// If the connection fails log the error and return -1.
int create_socket(std::string HOST, int PORT) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return -1;
    }

    sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    
    if (inet_pton(AF_INET, HOST.c_str(), &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
        return -1;
    }

    // Connect to server
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection Failed" << std::endl;
        return -1;
    }
    return sock;
}

// Send data over the socket and return the number of bytes sent.
ssize_t send_data(int sockfd, const std::string& data) {
    return send(sockfd, data.c_str(), data.size(), 0);
}

// Receive data from the socket and store it in the provided buffer. Return the number of bytes received.
ssize_t receive_data(int sockfd, char* buffer, size_t buffer_size) {
    memset(buffer, 0, buffer_size);
    return recv(sockfd, buffer, buffer_size - 1, 0);
}