#ifndef ATTITUDE_H
#define ATTITUDE_H

#define BUFLEN 512  //Max length of buffer
#define PORT 8081   //The port on which to send data
#define SERVER "10.0.0.2"

typedef struct{
  float rc_channels[8];
  float armed;
  float rc_lost;
  uint64_t timestamp;
} rc_channels;

#endif
