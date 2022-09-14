#ifndef ATTITUDE_H
#define ATTITUDE_H

#define BUFLEN 512  //Max length of buffer
#define PORT 8082   //The port on which to send data
#define SERVER "10.0.0.2"

typedef struct{
  float roll_s;
  float pitch_s;
  float yaw_s;
  float quat[4];
  float att_control[3];
  uint64_t timestamp;
} attitudeValues;

#endif
