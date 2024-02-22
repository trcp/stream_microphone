# stream_microphone
This package enables the input of audio data from another computer's microphone via a network connection to the input of a different computer.

## Installation
```bash
$ sudo apt install libpulse-dev
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src && catkin_init_workspace
$ git clone https://github.com/trcp/stream_microphone.git ~/catkin_ws/src/stream_microphone
$ cd catkin_ws/ && catkin_make
```

## RUN
### PC1
```bash
$ rosrun stream_microphone sender_node <DESTINATION IP>
```

### PC2
```bash
$ rosrun stream_microphone receiver_node
```
