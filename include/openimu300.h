#include <iostream>
#include <dw/sensors/canbus/CAN.h>
#include <dw/sensors/imu/IMU.h>
#include <map>
#include <vector>
#include <algorithm>
using namespace std;

typedef struct{
  uint16_t packetRate;
  uint16_t packetType;
  uint16_t orientation;
  uint16_t rateLPF;
  uint16_t accelLPF;
} imuParameters_t;

typedef enum{
  paramPACKET_RATE    = 0,
  paramPACKET_TYPE    = 1,
  paramORIENTATION    = 2,
  paramRATE_LPF       = 3,
  paramACCEL_LPF      = 4,
  // Add New parameteres here
  paramMAX_IMU_PARAMS,
}IMU_PARAM_NAME_t;

class IMU
{
  public:

    virtual ~IMU(){};

    virtual void init(vector<string> *paramsString, imuParameters_t *params) = 0;

    virtual void getConfigPacket(IMU_PARAM_NAME_t param, uint16_t paramVal, dwCANMessage *packet) = 0;

    virtual bool isValidMessage(uint32_t message_id) = 0;

    virtual bool parseDataPacket(dwCANMessage packet, dwIMUFrame *IMUframe) = 0;

  private:
};
