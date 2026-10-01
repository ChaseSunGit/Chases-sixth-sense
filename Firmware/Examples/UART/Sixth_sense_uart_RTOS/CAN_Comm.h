#ifndef CAN_COMM_H
#define CAN_COMM_H

void CAN_init();
void CAN_send_sensor_data(float* quat, float roll, float pitch, float yaw);

#endif