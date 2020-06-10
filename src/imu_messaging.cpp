#include <imu_messaging.h>

// TODO (06/10/2020):
// 1. Support for hex and decimal both for parameter values
// 2. Add all configuration parameters
// 3. Need to add priodity byte to Configureation packets
// 4. findDataPacket() should be improved to O(1) from O(n).
//      There is a significant overhead as it gets called at 200hz
// 5. Units for each IMUFrame needs to be verified atleast once
// 6. Add const keyword as much as possible
// 7. Restructoring (Open to extension close to modification)

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


const vector<string> paramNames = {"packet-rate=","packet-type=","orientation=","rateLPF=","accelLPF="};

const imuParameters_t defaultParams = {
          .packetRate   = 1,
          .packetType   = 1,
          .orientation  = 0,
          .rateLPF      = 0x0000,
          .accelLPF     = 0,
};

//----------------------------------------------------------------------------//

void OpenIMU300::getPakcetIdentifiers(uint32_t message_id, uint8_t *pf, uint8_t *ps)
{
  *pf = (uint8_t)((0x00FF0000 & message_id) >> 16);
  *ps = (uint8_t)((0x0000FF00 & message_id) >> 8);
}

//----------------------------------------------------------------------------//

imuMessages OpenIMU300::findDataPacket(uint8_t pf, uint8_t ps)
{
  for(int i = 0; i < static_cast<int>(IMU300pgnList.size()); i++)
  {
    if((IMU300pgnList[i].type & DATA_PACKET) != 0)
    {
      if(IMU300pgnList[i].PF == pf && IMU300pgnList[i].PS == ps)
      {
        return static_cast<imuMessages>(i);
      }
    }
  }
  return MAX_PGN;
}

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

void OpenIMU300::init(vector<string> *paramsString, imuParameters_t *params)
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

  *paramsString = paramNames;
  *params = defaultParams;

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

  uint8_t pf = 0, ps = 0;

  getPakcetIdentifiers(message_id, &pf, &ps);

  auto map_itr = PGNMap.find(pf);
  if(map_itr == PGNMap.end())             // PF key not available?
    return false;

  auto vec_itr = find(map_itr->second.begin(), map_itr->second.end(), ps);
  if(vec_itr == map_itr->second.end())
    return false;

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

bool OpenIMU300::parseDataPacket(dwCANMessage packet, dwIMUFrame *frame)
{
  uint8_t pf = 0, ps = 0;

  getPakcetIdentifiers(packet.id, &pf, &ps);

  // TODO: Needs improvement, change from O(n) to O(1)
  imuMessages dataPacketType = findDataPacket(pf, ps);

  switch(dataPacketType)
  {
    case ANGULAR_RATE_PT:
    {
        auto ptr = reinterpret_cast<const angularRate*>(packet.data);
        frame->turnrate[0] = static_cast<float32_t>(ptr->roll_rate) * (1/128.0) - 250.0;
        frame->turnrate[1] = static_cast<float32_t>(ptr->pitch_rate) * (1/128.0) - 250.0;
        frame->turnrate[2] = static_cast<float32_t>(ptr->yaw_rate) * (1/128.0) - 250.0;
        frame->flags |= DW_IMU_ROLL_RATE | DW_IMU_PITCH_RATE | DW_IMU_YAW_RATE;
        break;
    }

    case SSI1_PT:
    {
        auto ptr = reinterpret_cast<const slopeSensor*>(packet.data);
        frame->turnrate[0] = static_cast<float32_t>(ptr->roll) * (1/32768) - 250.0;
        frame->turnrate[1] = static_cast<float32_t>(ptr->pitch) * (1/32768) - 250.0;
        frame->turnrate[2] = 0;
        frame->flags |= DW_IMU_ROLL | DW_IMU_PITCH;
        break;
    }

    case ACCEL_PT :
    {
        auto ptr = reinterpret_cast<const accelSensor*>(packet.data);
        frame->acceleration[0] = static_cast<float32_t>(ptr-> acceleration_x) * 0.01f - 320.0;
        frame->acceleration[1] = static_cast<float32_t>(ptr-> acceleration_y) * 0.01f - 320.0;
        frame->acceleration[2] = static_cast<float32_t>(ptr-> acceleration_z) * 0.01f - 320.0;
        frame->flags |= DW_IMU_ACCELERATION_X | DW_IMU_ACCELERATION_Y | DW_IMU_ACCELERATION_Z;
        break;
    }

    case MAGNETOMETER_PT:
    {
        auto ptr = reinterpret_cast<const magSensor*>(packet.data);
        frame->magnetometer[0] = static_cast<float32_t>(ptr->mag_x) * 0.0002f;
        frame->magnetometer[1] = static_cast<float32_t>(ptr->mag_y) * 0.0002f;
        frame->magnetometer[2] = static_cast<float32_t>(ptr->mag_z) * 0.0002f;
        frame->flags |= DW_IMU_MAGNETOMETER_X | DW_IMU_MAGNETOMETER_Y | DW_IMU_MAGNETOMETER_Z;
        break;
    }
    default:
      return false;
  }

  return true;
}

//----------------------------------------------------------------------------//
