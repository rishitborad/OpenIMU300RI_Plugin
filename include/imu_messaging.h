#include <iostream>
#include <dw/sensors/canbus/CAN.h>
#include <map>
#include <vector>
#include <algorithm>
using namespace std;

typedef enum{
  NONE                  = 0,
  REQUEST_PACKET        = 1,
  CONFIGURATION_PACKET  = 2,
  REQ_CONFIG_PACKET     = 3,
  DATA_PACKET           = 4,
  REQ_DATA_PACKET       = 5,
}PACKET_TYPE_t;

struct pgn{
  PACKET_TYPE_t type;
  uint8_t PF;
  uint8_t PS;
  pgn(){type = NONE, PF = 0; PS = 0;}
  pgn(PACKET_TYPE_t type, uint8_t pf, uint8_t ps): type(type), PF(pf), PS(ps){}
};

typedef enum{
  GET_PACKET          = 0,
  ECU_ID              = 1,
  SOFTWARE_VER        = 2,
  RESET_ALGORITHM     = 3,
  SAVE_CONFIGURAITON  = 4,
  PACKET_RATE         = 5,
  PACKET_TYPE         = 6,
  FILTER_FREQ         = 7,
  ORIENTATION         = 8,
  MAG_ALIGNMENT       = 9,
  LEVER_ARM           = 10,
  BOPS_BANK0          = 11,
  BOPS_BANK1          = 12,
  SSI1_PT             = 13,
  ANGULAR_RATE_PT     = 14,
  ACCEL_PT            = 15,
  MAGNETOMETER_PT     = 16,
  MAX_PGN             = 17,
}imuMessages;

typedef enum{
  paramPACKET_RATE    = 0,
  paramPACKET_TYPE    = 1,
  paramORIENTATION    = 2,
  paramRATE_LPF       = 3,
  paramACCEL_LPF      = 4,
  paramMAX_IMU_PARAMS = 5,
}IMU_PARAM_NAME_t;

// TODO: Change this class name to Aceinna IMU
class IMUMessaging{
public:
  //IMUMessaging();
  virtual ~IMUMessaging(){};
  virtual void init() = 0;
  virtual void getConfigPacket(IMU_PARAM_NAME_t param, uint16_t paramVal, dwCANMessage *packet) = 0;
  virtual bool isValidMessage(uint32_t message_id/*, PACKET_TYPE_t *type*/) = 0;
  virtual void parser() = 0;
  //virtual void createPacket() = 0;
private:

};

class OpenIMU300 : public IMUMessaging
{
public:
  OpenIMU300();
  OpenIMU300(uint8_t srcAddr, uint8_t destAddr = 0x80);
  virtual ~OpenIMU300() override;
  virtual void init() override;
  virtual void getConfigPacket(IMU_PARAM_NAME_t param, uint16_t paramVal, dwCANMessage *packet) override;
  virtual bool isValidMessage(uint32_t message_id/*, PACKET_TYPE_t *type*/) override;
  virtual void parser() override;
  //virtual void createPacket() override;
private:
  void createPacket();
  //map<PF,Vector<PS,PS,PS>>
  map<uint8_t,vector<uint8_t>> PGNMap;
  uint8_t SRCAddress;
  uint8_t ECUAddress;
};
