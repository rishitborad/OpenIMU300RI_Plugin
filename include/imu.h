#include <iostream>
#include <dw/sensors/canbus/CAN.h>
#include <dw/sensors/imu/IMU.h>
#include <map>
#include <vector>
#include <algorithm>
using namespace std;

class IMU
{
  public:

    virtual ~IMU(){};

    virtual void init(string paramsString, std::vector<dwCANMessage> & configMessages) = 0;

    virtual bool isValidMessage(uint32_t message_id) = 0;

    virtual bool parseDataPacket(dwCANMessage packet, dwIMUFrame *IMUframe) = 0;

  private:
};
