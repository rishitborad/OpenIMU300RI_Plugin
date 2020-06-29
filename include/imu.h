#include <iostream>
#include <dw/sensors/canbus/CAN.h>
#include <dw/sensors/imu/IMU.h>
#include <vector>
#include <algorithm>
using namespace std;

class IMU
{
  public:

    virtual ~IMU(){};

    virtual bool init(string paramsString, dwCANMessage **messages, uint8_t *count) = 0;

    virtual bool isValidMessage(uint32_t message_id) = 0;

    virtual bool parseDataPacket(dwCANMessage packet, dwIMUFrame *IMUframe) = 0;

    virtual void getSensorResetMessage(dwCANMessage *packet) = 0;

  private:
};
