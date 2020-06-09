#include <iostream>
#include <map>
#include <vector>
using namespace std;

struct pgn{
  uint8_t PF;
  uint8_t PS;
  pgn(){PF = 0; PS = 0;}
  pgn(uint8_t pf, uint8_t ps): PF(pf), PS(ps){}
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


class IMUMessaging{
public:
  //IMUMessaging();
  virtual ~IMUMessaging(){};
  virtual bool isValidMessage(uint32_t message_id) = 0;
  virtual void parser() = 0;
  virtual void createPacket() = 0;
private:

};

class OpenIMU300 : public IMUMessaging
{
public:
  OpenIMU300();
  virtual ~OpenIMU300() override;
  virtual bool isValidMessage(uint32_t message_id) override;
  virtual void parser() override;
  virtual void createPacket() override;
private:
  map<uint8_t,vector<uint8_t>> messagePGN;
};
