#include <iostream>
#include <unistd.h>
#include <pulse/simple.h>
#include <pulse/pulseaudio.h>
#include <arpa/inet.h>

#include "ros/ros.h"

#define SERVER_PORT 12345

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <server_ip>" << std::endl;
        return -1;
    }

    const char* SERVER_IP = argv[1];

    ros::init(argc, argv, "audio_sender_node");  // ROSノードの初期化

    pa_simple *s = NULL;
    pa_sample_spec ss;
    int error;

    ss.format = PA_SAMPLE_S16LE;
    ss.channels = 2;
    ss.rate = 44100;

    s = pa_simple_new(NULL, "Stream", PA_STREAM_RECORD, NULL, "record", &ss, NULL, NULL, &error);
    if (!s) {
        std::cerr << "pa_simple_new() failed: " << pa_strerror(error) << std::endl;
        return -1;
    }

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        perror("socket");
        return -1;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    while (ros::ok()) {
        uint8_t buf[1024];

        if (pa_simple_read(s, buf, sizeof(buf), &error) < 0) {
            std::cerr << "pa_simple_read() failed: " << pa_strerror(error) << std::endl;
            break;
        }

        sendto(sockfd, buf, sizeof(buf), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));

        ros::spinOnce();  // ROSのイベント処理
    }

    pa_simple_free(s);
    close(sockfd);
    return 0;
}
