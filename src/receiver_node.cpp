#include <iostream>
#include <unistd.h>
#include <pulse/pulseaudio.h>
#include <pulse/thread-mainloop.h>
#include <pulse/simple.h>
#include <arpa/inet.h>

#include "ros/ros.h"
#include "std_srvs/SetBool.h"

#define SERVER_PORT 12345

class AudioStreamer {
public:
  AudioStreamer() : s(nullptr), sockfd(-1) {

    ss.format = PA_SAMPLE_S16LE;
    ss.channels = 2;
    ss.rate = 44100;

    s = pa_simple_new(NULL, "Stream", PA_STREAM_PLAYBACK, NULL, "playback", &ss, NULL, NULL, &error);
    if (!s) {
      std::cerr << "pa_simple_new() failed: " << pa_strerror(error) << std::endl;
      std::exit(-1);
    }

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
      perror("socket");
      std::exit(-1);
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
      perror("bind");
      std::exit(-1);
    }

    pa = pa_threaded_mainloop_new();
    if (!pa) {
      std::cerr << "Failed to initialize pulseaudio" << std::endl;
      std::exit(1);
    }

    if (pa_threaded_mainloop_start(pa) < 0) {
      std::cerr << "Failed to start the main loop" << std::endl;
      std::exit(1);
    }

    mainloop_api = pa_threaded_mainloop_get_api(pa);
    context = pa_context_new(mainloop_api, "default");

    pa_threaded_mainloop_lock(pa);
    int err;
    err = pa_context_connect(context, NULL, PA_CONTEXT_NOFLAGS, NULL);
    if (err < 0) {
      pa_threaded_mainloop_unlock(pa);
      std::cerr << "Could not connect to the server (" << pa_strerror(err) << ")" << std::endl;
      std::exit(1);
    }
    pa_context_set_state_callback(context, context_state_callback, all_idx);

    pa_threaded_mainloop_unlock(pa);

    signal(SIGINT, signal_handler);
  }
  
  // Function to return a pointer to the instance
  static AudioStreamer* instancePointer;
  static AudioStreamer* getInstancePointer() {
    return instancePointer;
  }

  bool mute_mic = false;
  bool triggerMuteCallback(std_srvs::SetBool::Request &req,
                           std_srvs::SetBool::Response &res) {
    ROS_INFO("Mute microphone");
    mute_mic = req.data;
    res.success = true;
    res.message = "Trigger successful";
    return true;
  }  
  
  void run() {
    for (;;) {
      step();
    }
  }

  void step() {
    pa_threaded_mainloop_lock(pa);

    // std::cout << "loop-> " << receive_data << std::endl;
    if (!mute_mic) {
      ssize_t len = recvfrom(sockfd, buf, sizeof(buf), 0, (struct sockaddr*)&client_addr, &addr_len);
      if (len == -1) {
        perror("recvfrom");
      }
      if (pa_simple_write(s, buf, len, &error) < 0) {
        std::cerr << "pa_simple_write() failed: " << pa_strerror(error) << std::endl;
      }
    } else {
      std::cout << "mute" << std::endl;
    }
    pa_threaded_mainloop_unlock(pa);
  }

  static void signal_handler(int signal) {
    AudioStreamer* audioStreamerPtr = getInstancePointer();
    pa_threaded_mainloop_lock(audioStreamerPtr->pa);
    if (audioStreamerPtr) {
      std::cout << "ctrl + c called" << std::endl;
      std::cout << "exit loop "
                << audioStreamerPtr->all_idx[0] << ", "
                << audioStreamerPtr->all_idx[1]
                << std::endl;

      // TOOD: Fix to use pulseaudio API
      int result;
      char command[50];
      snprintf(command, sizeof(command), "pactl unload-module %d", audioStreamerPtr->all_idx[0]);
      result = system(command);

      snprintf(command, sizeof(command), "pactl unload-module %d", audioStreamerPtr->all_idx[1]);
      result = system(command);
      std::cout << "here" << std::endl;
      // audioStreamerPtr->cleanup();

      ros::shutdown();
      std::exit(1);
    }
  }

  ~AudioStreamer() {
    cleanup();
  }
  

private:
  pa_simple *s;
  pa_sample_spec ss;
  int error;
  uint8_t buf[1024];

  int sockfd;
  struct sockaddr_in server_addr, client_addr;
  socklen_t addr_len = sizeof(client_addr);

  pa_threaded_mainloop *pa;
  pa_mainloop_api *mainloop_api;
  pa_context *context;

  int all_idx[2] = {0, 0};

  static void load_module_callback(pa_context *c, uint32_t idx, void *userdata) {
    int *v = (int*)userdata;

    assert(c);
    if (idx == PA_INVALID_INDEX) {
      std::cerr << "Bad index" << std::endl;
      return;
    }
    int _idx = idx;
    *v = idx;
  }

  static void context_state_callback(pa_context *c, void *userdata) {
    pa_operation *op;

    int *arr = (int*)userdata;

    if (pa_context_get_state(c) == PA_CONTEXT_READY) {
      op = pa_context_load_module(c, "module-null-sink", "sink_name=virtual_speaker sink_properties=device.description=virtual_speaker", load_module_callback, &arr[0]);
      if (op) {
        pa_operation_unref(op);
      } else {
        std::cerr << "Failed to load module-null-sink" << std::endl;
      }

      op = pa_context_load_module(c, "module-remap-source", "master=virtual_speaker.monitor source_name=virtual_mic source_properties=device.description=virtual_mic", load_module_callback, &arr[1]);
      if (op) {
        pa_operation_unref(op);
      } else {
        std::cerr << "Failed to load module-remap-source" << std::endl;
      }
    }
  }

  void cleanup() {
    pa_context_disconnect(context);
    pa_context_unref(context);
    pa_threaded_mainloop_free(pa);
    close(sockfd);
  }
};

AudioStreamer *AudioStreamer::instancePointer = nullptr;
int main(int argc, char *argv[]) {
  AudioStreamer streamer;
  AudioStreamer::instancePointer = &streamer;
  ros::init(argc, argv, "audio_receiver_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("trigger_mute",  &AudioStreamer::triggerMuteCallback, &streamer);
  ROS_INFO("Ready to receive.");

  while (ros::ok) {
    ros::spinOnce(); 
    streamer.step();
  }
  std::cout << "end" << std::endl;
  
  return 0;
}
