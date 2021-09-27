#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_PROFILE_ACCELERATION     108
#define ADDR_X_PROFILE_VELOCITY         112
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_POSITION         132

// Limit values (XM430-W210-T)
#define LIMIT_X_MAX_VELOCITY            240

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4

#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0

#define DXL_ID                          1       // motor ID
//#define DXL_ID2                         2       // motor ID

#define BAUDRATE                        1000000 // baud rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

class MotorDriver
{
 public:
  MotorDriver();
  ~MotorDriver();
  bool init(void);
  void closeDynamixel(void);
  bool setTorque(uint8_t id, bool onoff);
  bool readEncoder(int32_t& value/*, int32_t& value2*/);
  bool VelControl(int8_t id, int64_t value);
  bool PosControl(int8_t id, int64_t value);

 private:
  uint32_t baudrate_;
  float  protocol_version_;
  uint8_t dxl_id_;//, dxl_id2_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupBulkWrite *groupBulkWrite_;
  dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
  dynamixel::GroupSyncRead *groupSyncReadEncoder_;
};

#endif // MOTOR_DRIVER_H_
