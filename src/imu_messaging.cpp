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

  OpenIMU300::OpenIMU300()
  {
    for(int i = 0; i < (int)pgn.size(); i++)
    {
      auto idx = messagePGN.find(pgn[i].PF);
      if(idx != messagePGN.end())
      {
        cout<<"In map\r\n";
        idx->second.push_back(pgn[i].PS);
      }
      else{
        cout<<"Not In map\r\n";
        messagePGN.insert({pgn[i].PF, std::vector<uint8_t>{pgn[i].PS}});
      }
    }
  }

  OpenIMU300::~OpenIMU300()
  {

  }

  bool OpenIMU300::isValidMessage(uint32_t message_id)
  {

    for(int i = 0; i < (int)pgn.size(); i++)
    {
      auto idx = messagePGN.find(pgn[i].PF);
      if(idx != messagePGN.end())
      {
        cout<<"In map\r\n";
        idx->second.push_back(pgn[i].PS);
      }
      else{
        cout<<"Not In map\r\n";
        messagePGN.insert({pgn[i].PF, std::vector<uint8_t>{pgn[i].PS}});
      }
    }
    if((message_id & 0x000000FF) == 0x00000080)
    {
      uint8_t PF = (uint8_t)((0x00FF0000 & message_id) >> 16);
      uint8_t PS = (uint8_t)((0x0000FF00 & message_id) >> 8);
      (void)PF;
      (void)PS;

    }
    cout<< (int)pgn.size()<<"ISVALID\r\n";
    for(auto i = messagePGN.begin(); i != messagePGN.end(); i++)
    {
      printf("PGN: %d\r\n", i->first);
    }
    return false;
  }

  void OpenIMU300::parser()
  {
    return;
  }

  void OpenIMU300::createPacket()
  {
    return;
  }
