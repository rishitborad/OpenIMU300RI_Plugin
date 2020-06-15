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
  uint8_t saveConfig;
  uint8_t resetAlgo;
} imuParameters_t;

typedef enum{
  paramPACKET_RATE    = 0,
  paramPACKET_TYPE    = 1,
  paramORIENTATION    = 2,
  paramRATE_LPF       = 3,
  paramACCEL_LPF      = 4,
  paramSAVE_CONFIG    = 5,
  paramRESET_ALGO     = 6,
  // Add New parameteres here
  paramMAX_IMU_PARAMS,
}configParams;

class IMU
{
  public:

    virtual ~IMU(){};

    virtual void init(vector<string> *paramsString, imuParameters_t *params) = 0;

    virtual bool getConfigPacket(configParams param, uint16_t paramVal, dwCANMessage *packet) = 0;

    virtual bool isValidMessage(uint32_t message_id) = 0;

    virtual bool parseDataPacket(dwCANMessage packet, dwIMUFrame *IMUframe) = 0;

  private:
};
