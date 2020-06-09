#include <imu_messaging.h>


static vector<pgn> pgn =  { {.PF = 234, .PS = 255}   //GET_PACKET
                    ,{.PF = 253, .PS = 197}   //ECU_ID
                    ,{.PF = 254, .PS = 218}   //SOFTWARE_VER
                    ,{.PF = 255, .PS = 80}    //RESET_ALGORITHM
                    ,{.PF = 255, .PS = 81}    //SAVE_CONFIGURAITON
                    ,{.PF = 255, .PS = 85}    //PACKET_RATE
                    ,{.PF = 255, .PS = 86}    //PACKET_TYPE
                    ,{.PF = 255, .PS = 87}    //FILTER_FREQ
                    ,{.PF = 255, .PS = 88}    //ORIENTATION
                    ,{.PF = 255, .PS = 94}    //MAG_ALIGNMENT
                    ,{.PF = 255, .PS = 95}    //LEVER_ARM
                    ,{.PF = 255, .PS = 240}   //BOPS_BANK0
                    ,{.PF = 255, .PS = 241}   //BOPS_BANK1
                    ,{.PF = 240, .PS = 41}    //SSI1_PT
                    ,{.PF = 240, .PS = 42}    //ANGULAR_RATE_PT
                    ,{.PF = 240, .PS = 45}    //ACCEL_PT
                    ,{.PF = 255, .PS = 106}   //MAGNETOMETER_PT
                   };

//----------------------------------------------------------------------------//

OpenIMU300::OpenIMU300()
{

}

//----------------------------------------------------------------------------//

OpenIMU300::~OpenIMU300()
{

}

//----------------------------------------------------------------------------//

void OpenIMU300::init()
{
  for(int i = 0; i < (int)pgn.size(); i++)
  {
    auto idx = PGN_list.find(pgn[i].PF);
    if(idx != PGN_list.end())
    {
      idx->second.push_back(pgn[i].PS);
    }
    else{
      PGN_list.insert({pgn[i].PF, std::vector<uint8_t>{pgn[i].PS}});
    }
  }

  #if 0
  for(auto i = PGN_list.begin(); i != PGN_list.end(); i++)
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

  auto map_itr = PGN_list.find(PF);
  if(map_itr == PGN_list.end())             // PF key not available?
    return false;

  auto vec_itr = find(map_itr->second.begin(), map_itr->second.end(), PS);
  if(vec_itr == map_itr->second.end())
    return false;

  printf("Valid: %d %d\r\n", PF, PS);
  return true;
}

//----------------------------------------------------------------------------//

void OpenIMU300::parser()
{
  return;
}

//----------------------------------------------------------------------------//


void OpenIMU300::createPacket()
{
  return;
}

//----------------------------------------------------------------------------//
