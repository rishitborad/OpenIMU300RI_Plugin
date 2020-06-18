#include <openimu300_plugin.h>

// TODO (06/10/2020):
// 1. Support for hex and decimal both for parameter values
// 2. Set Bank of PS number configuration not supported
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


const vector<string> paramNames = { "packetRate=","packetType=","orientation=",
                                    "rateLPF=","accelLPF=","saveConfig=","resetAlgo=",
                                    "setBank0PS0","setBank0PS1","setBank0PS2","setBank0PS3",
                                    "setBank0PS4","setBank1PS0","setBank1PS1","setBank1PS2",
                                    "setBank1PS3","setBank1PS4","setBank1PS5","setBank1PS6",
                                  };

const vector<uint8_t> validPacketRates = {0,1,2,4,5,10,20,25,50};
const vector<uint8_t> validPacketTypes = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
const vector<uint8_t> validCutoffFreqs = {0,2,5,10,25,40,50};
const vector<uint16_t> validOrientations = {0x0000, 0x0009, 0x0023, 0x002A,
                                            0x0041, 0x0048, 0x0062, 0x006B,
                                            0x0085, 0x008C, 0x0092, 0x009B};

const imuParameters_t defaultParams = {
          .packetRate   = 1,
          .packetType   = 1,
          .orientation  = 0x0000,
          .rateLPF      = 0,
          .accelLPF     = 0,
          .saveConfig   = 0,
          .resetAlgo    = 0
};
//----------------------------------------------------------------------------//

void OpenIMU300::getPacketIdentifiers(uint32_t message_id, uint8_t *pf, uint8_t *ps)
{
  *pf = (uint8_t)((0x00FF0000 & message_id) >> 16);
  *ps = (uint8_t)((0x0000FF00 & message_id) >> 8);
}

//----------------------------------------------------------------------------//

imuMessages OpenIMU300::findDataPacket(uint8_t pf, uint8_t ps)
{
  for(size_t i = 0; i < IMU300pgnList.size(); i++)
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

bool OpenIMU300::getParameterVal(std::string searchString, std::string userString,  uint16_t* value)
{
  std::string param;
  size_t pos = userString.find(searchString);
  *value = 0;
  if (pos != std::string::npos)
  {
      param   = userString.substr(pos);
      pos     = param.find_first_of(",");
      param   = param.substr(searchString.length(), pos);
      int val = static_cast<uint16_t>(stoi(param));
      // Clamp negative values to zero
      if(val < 0)
      {
        val = 0;
      }
      *value = val;
      return true;
  }
  return false;
}

//----------------------------------------------------------------------------//

void OpenIMU300::getParams(std::string userString, vector<dwCANMessage> &configMessages)
{
  for(size_t i = 0; i < paramNames.size(); i++)
  {
    bool isValidParam = true;
    uint16_t val = 0;
    dwCANMessage message;
    if(getParameterVal(paramNames[i], userString, &val))
    {
      printf("PARAM: %lu %d\r\n", i, val);
      switch(static_cast<IMUPARAM_t>(i))
      {
        case IMUPARAM_t::PARAM_PACKET_RATE: imuParameter.packetRate  = val; break;
        case IMUPARAM_t::PARAM_PACKET_TYPE: imuParameter.packetType  = val; break;
        case IMUPARAM_t::PARAM_ORIENTATION: imuParameter.orientation = val; break;
        case IMUPARAM_t::PARAM_RATE_LPF:    imuParameter.rateLPF     = val; break;
        case IMUPARAM_t::PARAM_ACCEL_LPF:   imuParameter.accelLPF    = val; break;
        case IMUPARAM_t::PARAM_SAVE_CONFIG: imuParameter.saveConfig  = val; break;
        case IMUPARAM_t::PARAM_RESET_ALGO:  imuParameter.resetAlgo   = val; break;
        default:
          isValidParam = false;
          break;
      }

      if(isValidParam)
      {
        getConfigPacket(static_cast<IMUPARAM_t>(i), val, &message);
        configMessages.push_back(message);
      }
    }
  }
}

//----------------------------------------------------------------------------//

OpenIMU300::OpenIMU300()
{ }

//----------------------------------------------------------------------------//

OpenIMU300::OpenIMU300(uint8_t srcAddr, uint8_t destAddr)
: SRCAddress(srcAddr)
, ECUAddress(destAddr)
, imuParameter(defaultParams)
{ }

//----------------------------------------------------------------------------//

OpenIMU300::~OpenIMU300()
{ }

//----------------------------------------------------------------------------//

void OpenIMU300::init(string paramsString, vector<dwCANMessage> &configMessages)
{

  getParams(paramsString, configMessages);

  for(size_t i = 0; i < IMU300pgnList.size(); i++)
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

  uint8_t pf = 0, ps = 0;

  getPacketIdentifiers(message_id, &pf, &ps);

  auto map_itr = PGNMap.find(pf);
  if(map_itr == PGNMap.end())             // PF key not available?
    return false;

  auto vec_itr = find(map_itr->second.begin(), map_itr->second.end(), ps);
  if(vec_itr == map_itr->second.end())
    return false;

  return true;
}

//----------------------------------------------------------------------------//

bool OpenIMU300::getConfigPacket(IMUPARAM_t param, uint16_t paramVal, dwCANMessage *packet)
{
  switch(static_cast<IMUPARAM_t>(param))
  {
    case IMUPARAM_t::PARAM_PACKET_RATE:
      {
        if(find(validPacketRates.begin(), validPacketRates.end(), paramVal) == validPacketRates.end())
        {
          return false;     // paramVal is not valid
        }
        pgn info = IMU300pgnList[PACKET_RATE];
        if((info.type & PACKET_TYPE_t::CONFIGURATION_PACKET) != 0)
        {
          packet->id = 0x18000000;
          packet->id |= (info.PF << 16);
          packet->id |= (info.PS << 8);
          packet->id |= static_cast<uint8_t>(SRCAddress);
          packet->size = 2;
          packet->timestamp_us = 0;
          packet->data[0] = this->ECUAddress;
          packet->data[1] = static_cast<uint8_t>(paramVal & 0xFF);
          return true;
        }
      }
      break;
    case IMUPARAM_t::PARAM_PACKET_TYPE:
      {
        if(find(validPacketTypes.begin(), validPacketTypes.end(), paramVal) == validPacketTypes.end())
        {
          return false;     // paramVal is not valid
        }
        pgn info = IMU300pgnList[PACKET_TYPE];
        if((info.type & PACKET_TYPE_t::CONFIGURATION_PACKET) != 0)
        {
          packet->id = 0x18000000;
          packet->id |= (info.PF << 16);
          packet->id |= (info.PS << 8);
          packet->id |= static_cast<uint8_t>(SRCAddress);
          packet->size = 3;
          packet->timestamp_us = 0;
          packet->data[0] = this->ECUAddress;
          packet->data[1] = static_cast<uint8_t>(paramVal & 0xFF);
          packet->data[2] = 0;
          return true;
        }
      }
      break;
    case IMUPARAM_t::PARAM_ORIENTATION:
      {
        if(find(validOrientations.begin(), validOrientations.end(), paramVal) == validOrientations.end())
        {
          return false;     // paramVal is not valid
        }

        pgn info = IMU300pgnList[ORIENTATION];
        if((info.type & PACKET_TYPE_t::CONFIGURATION_PACKET) != 0)
        {
          packet->id = 0x18000000;
          packet->id |= (info.PF << 16);
          packet->id |= (info.PS << 8);
          packet->id |= static_cast<uint8_t>(SRCAddress);
          packet->size = 3;
          packet->timestamp_us = 0;
          packet->data[0] = this->ECUAddress;
          packet->data[1] = static_cast<uint8_t>((paramVal & 0xFF00) >> 8);   // Orientation MSB
          packet->data[2] = static_cast<uint8_t>(paramVal & 0x00FF);          // Orientation LSB
          return true;
        }
      }
      break;
    case IMUPARAM_t::PARAM_RATE_LPF:
      {
        if(find(validCutoffFreqs.begin(), validCutoffFreqs.end(), paramVal) == validCutoffFreqs.end())
        {
          return false;     // paramVal is not valid
        }
        pgn info = IMU300pgnList[FILTER_FREQ];
        if((info.type & PACKET_TYPE_t::CONFIGURATION_PACKET) != 0)
        {
          packet->id = 0x18000000;
          packet->id |= (info.PF << 16);
          packet->id |= (info.PS << 8);
          packet->id |= static_cast<uint8_t>(SRCAddress);
          packet->size = 3;
          packet->timestamp_us = 0;
          packet->data[0] = this->ECUAddress;
          packet->data[1] = static_cast<uint8_t>(paramVal);   // Rate LPF byte
          packet->data[2] = 0;                                // Accel LPF byte
          return true;
        }
      }
      break;
    case IMUPARAM_t::PARAM_ACCEL_LPF:
      {
        if(find(validCutoffFreqs.begin(), validCutoffFreqs.end(), paramVal) == validCutoffFreqs.end())
        {
          return false;     // paramVal is not valid
        }
        pgn info = IMU300pgnList[FILTER_FREQ];
        if((info.type & PACKET_TYPE_t::CONFIGURATION_PACKET) != 0)
        {
          packet->id = 0x18000000;
          packet->id |= (info.PF << 16);
          packet->id |= (info.PS << 8);
          packet->id |= static_cast<uint8_t>(SRCAddress);
          packet->size = 3;
          packet->timestamp_us = 0;
          packet->data[0] = this->ECUAddress;
          packet->data[1] = 0;                                // Rate LPF byte
          packet->data[2] = static_cast<uint8_t>(paramVal);   // Accel LPF byte
          return true;
        }
      }
        break;
    case IMUPARAM_t::PARAM_SAVE_CONFIG:
      {
        if(paramVal != 1)
        {
          return false;     // paramVal is not valid
        }

        pgn info = IMU300pgnList[SAVE_CONFIGURAITON];
        if((info.type & PACKET_TYPE_t::CONFIGURATION_PACKET) != 0)
        {
          packet->id = 0x18000000;
          packet->id |= (info.PF << 16);
          packet->id |= (info.PS << 8);
          packet->id |= static_cast<uint8_t>(SRCAddress);
          packet->size = 3;
          packet->timestamp_us = 0;
          packet->data[0] = 0;
          packet->data[1] = this->ECUAddress;
          packet->data[2] = 0;
          return true;
        }
      }
      break;
    case IMUPARAM_t::PARAM_RESET_ALGO:
      {
        if(paramVal != 1)
        {
          return false;     // paramVal is not valid
        }

        pgn info = IMU300pgnList[RESET_ALGORITHM];
        if((info.type & PACKET_TYPE_t::CONFIGURATION_PACKET) != 0)
        {
          packet->id = 0x18000000;
          packet->id |= (info.PF << 16);
          packet->id |= (info.PS << 8);
          packet->id |= static_cast<uint8_t>(SRCAddress);
          packet->size = 3;
          packet->timestamp_us = 0;
          packet->data[0] = 0;
          packet->data[1] = this->ECUAddress;
          packet->data[2] = 0;
          return true;
        }
      }
      break;
    default:
      break;
  }
  return false;
}

//----------------------------------------------------------------------------//

bool OpenIMU300::parseDataPacket(dwCANMessage packet, dwIMUFrame *frame)
{
  uint8_t pf = 0, ps = 0;

  getPacketIdentifiers(packet.id, &pf, &ps);

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
        frame->orientation[0] = static_cast<float32_t>(ptr->roll) * (1/32768) - 250.0;
        frame->orientation[1] = static_cast<float32_t>(ptr->pitch) * (1/32768) - 250.0;
        frame->orientation[2] = 0;
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
        frame->magnetometer[0] = ((static_cast<float32_t>(ptr->mag_x) * 0.00025f) - 8) * (100 /*To uTesla*/);
        frame->magnetometer[1] = ((static_cast<float32_t>(ptr->mag_y) * 0.00025f) - 8) * (100 /*To uTesla*/);
        frame->magnetometer[2] = ((static_cast<float32_t>(ptr->mag_z) * 0.00025f) - 8) * (100 /*To uTesla*/);
        frame->flags |= DW_IMU_MAGNETOMETER_X | DW_IMU_MAGNETOMETER_Y | DW_IMU_MAGNETOMETER_Z;
        break;
    }
    default:
      return false;
  }

  return true;
}

//----------------------------------------------------------------------------//
