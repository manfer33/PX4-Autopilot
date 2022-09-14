#ifndef ATTITUDE_H
#define ATTITUDE_H

#define BUFLEN 512  //Max length of buffer
#define PORT 8083   //The port on which to send data
#define SERVER "10.0.0.2"

typedef struct{
  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
  uint64_t timestamp;
} local_pos_values;

#endif
