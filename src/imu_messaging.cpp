#include <imu_messaging.h>


// Release Notes:
// 1. Only supports Decimal parameter values
// 2.

static vector<pgn> IMU300pgnList =  {
                     {.type = REQUEST_PACKET,         .PF = 234, .PS = 255}   //GET_PACKET
                    ,{.type = REQUEST_PACKET,         .PF = 253, .PS = 197}   //ECU_ID
                    ,{.type = REQUEST_PACKET,         .PF = 254, .PS = 218}   //SOFTWARE_VER
                    ,{.type = CONFIGURATION_PACKET,   .PF = 255, .PS = 80}    //RESET_ALGORITHM
                    ,{.type = CONFIGURATION_PACKET,   .PF = 255, .PS = 81}    //SAVE_CONFIGURAITON
                    ,{.type = REQ_CONFIG_PACKET,      .PF = 255, .PS = 85}    //PACKET_RATE
                    ,{.type = REQ_CONFIG_PACKET,      .PF = 255, .PS = 86}    //PACKET_TYPE
                    ,{.type = REQ_CONFIG_PACKET,      .PF = 255, .PS = 87}    //FILTER_FREQ
                    ,{.type = REQ_CONFIG_PACKET,      .PF = 255, .PS = 88}    //ORIENTATION
                    ,{.type = CONFIGURATION_PACKET,   .PF = 255, .PS = 94}    //MAG_ALIGNMENT
                    ,{.type = REQ_CONFIG_PACKET,      .PF = 255, .PS = 95}    //LEVER_ARM
                    ,{.type = CONFIGURATION_PACKET,   .PF = 255, .PS = 240}   //BOPS_BANK0
                    ,{.type = CONFIGURATION_PACKET,   .PF = 255, .PS = 241}   //BOPS_BANK1
                    ,{.type = DATA_PACKET,            .PF = 240, .PS = 41}    //SSI1_PT
                    ,{.type = DATA_PACKET,            .PF = 240, .PS = 42}    //ANGULAR_RATE_PT
                    ,{.type = DATA_PACKET,            .PF = 240, .PS = 45}    //ACCEL_PT
                    ,{.type = DATA_PACKET,            .PF = 255, .PS = 106}   //MAGNETOMETER_PT
                   };

//----------------------------------------------------------------------------//

OpenIMU300::OpenIMU300()
{ }

//----------------------------------------------------------------------------//

OpenIMU300::OpenIMU300(uint8_t srcAddr, uint8_t destAddr)
: SRCAddress(srcAddr)
, ECUAddress(destAddr)
{ }

//----------------------------------------------------------------------------//

OpenIMU300::~OpenIMU300()
{ }

//----------------------------------------------------------------------------//

void OpenIMU300::init()
{
  for(int i = 0; i < (int)IMU300pgnList.size(); i++)
  {
    auto idx = PGNMap.find(IMU300pgnList[i].PF);
    if(idx != PGNMap.end())
    {
      idx->second.push_back(IMU300pgnList[i].PS);
    }
    else{
      PGNMap.insert({IMU300pgnList[i].PF, std::vector<uint8_t>{IMU300pgnList[i].PS}});
    }
  }

  #if 0
  for(auto i = PGNMap.begin(); i != PGNMap.end(); i++)
  {
    printf("PF: %d", i->first);
    for(int j = 0; j < (int)i->second.size(); j++)
    {
      printf(" %d", i->second[j]);
    }
    printf("\r\n");
  }
  #endif
}

//----------------------------------------------------------------------------//

bool OpenIMU300::isValidMessage(uint32_t message_id)
{
  if((message_id & 0x000000FF) != 0x00000080)
  {
    return false;
  }

  uint8_t PF = (uint8_t)((0x00FF0000 & message_id) >> 16);
  uint8_t PS = (uint8_t)((0x0000FF00 & message_id) >> 8);

  auto map_itr = PGNMap.find(PF);
  if(map_itr == PGNMap.end())             // PF key not available?
    return false;

  auto vec_itr = find(map_itr->second.begin(), map_itr->second.end(), PS);
  if(vec_itr == map_itr->second.end())
    return false;

  //printf("Valid: %d %d\r\n", PF, PS);
  return true;
}

//----------------------------------------------------------------------------//

void OpenIMU300::getConfigPacket(IMU_PARAM_NAME_t param, uint16_t paramVal, dwCANMessage *packet)
{
  switch(static_cast<IMU_PARAM_NAME_t>(param))
  {
    case IMU_PARAM_NAME_t::paramPACKET_RATE:
      {
        pgn info = IMU300pgnList[PACKET_RATE];
        if((info.type & PACKET_TYPE_t::CONFIGURATION_PACKET) != 0)
        {
          packet->id = 0x00000000;
          packet->id |= (info.PF << 16);
          packet->id |= (info.PS << 8);
          packet->id |= static_cast<uint8_t>(SRCAddress);
          packet->size = 2;
          packet->timestamp_us = 0;
          packet->data[0] = this->ECUAddress;
          packet->data[1] = static_cast<uint8_t>(paramVal & 0xFF);
        }
        return;
      }
    case IMU_PARAM_NAME_t::paramPACKET_TYPE:
      {
        pgn info = IMU300pgnList[PACKET_TYPE];
        if((info.type & PACKET_TYPE_t::CONFIGURATION_PACKET) != 0)
        {
          packet->id = 0x00000000;
          packet->id |= (info.PF << 16);
          packet->id |= (info.PS << 8);
          packet->id |= static_cast<uint8_t>(SRCAddress);
          packet->size = 3;
          packet->timestamp_us = 0;
          packet->data[0] = this->ECUAddress;
          packet->data[1] = static_cast<uint8_t>(paramVal & 0xFF);
          packet->data[2] = 0;
        }
        return;
      }
    case IMU_PARAM_NAME_t::paramORIENTATION:
      {
        pgn info = IMU300pgnList[ORIENTATION];
        if((info.type & PACKET_TYPE_t::CONFIGURATION_PACKET) != 0)
        {
          packet->id = 0x00000000;
          packet->id |= (info.PF << 16);
          packet->id |= (info.PS << 8);
          packet->id |= static_cast<uint8_t>(SRCAddress);
          packet->size = 3;
          packet->timestamp_us = 0;
          packet->data[0] = this->ECUAddress;
          packet->data[1] = static_cast<uint8_t>((paramVal & 0xFF00) >> 8);   // Orientation MSB
          packet->data[2] = static_cast<uint8_t>(paramVal & 0x00FF);          // Orientation LSB
        }
        return;
      }
    case IMU_PARAM_NAME_t::paramRATE_LPF:
      {
        pgn info = IMU300pgnList[FILTER_FREQ];
        if((info.type & PACKET_TYPE_t::CONFIGURATION_PACKET) != 0)
        {
          packet->id = 0x00000000;
          packet->id |= (info.PF << 16);
          packet->id |= (info.PS << 8);
          packet->id |= static_cast<uint8_t>(SRCAddress);
          packet->size = 3;
          packet->timestamp_us = 0;
          packet->data[0] = this->ECUAddress;
          packet->data[1] = static_cast<uint8_t>(paramVal);   // Rate LPF byte
          packet->data[2] = 0;                                // Accel LPF byte
        }
        return;
      }
    case IMU_PARAM_NAME_t::paramACCEL_LPF:
      {
        pgn info = IMU300pgnList[FILTER_FREQ];
        if((info.type & PACKET_TYPE_t::CONFIGURATION_PACKET) != 0)
        {
          packet->id = 0x00000000;
          packet->id |= (info.PF << 16);
          packet->id |= (info.PS << 8);
          packet->id |= static_cast<uint8_t>(SRCAddress);
          packet->size = 3;
          packet->timestamp_us = 0;
          packet->data[0] = this->ECUAddress;
          packet->data[1] = 0;                                // Rate LPF byte
          packet->data[2] = static_cast<uint8_t>(paramVal);   // Accel LPF byte
        }
        return;
      }
    default:
      break;
  }
}

//----------------------------------------------------------------------------//

void OpenIMU300::parser()
{
  return;
}

//----------------------------------------------------------------------------//

/*
void OpenIMU300::createPacket()
{

  return;
}
*/
//----------------------------------------------------------------------------//
