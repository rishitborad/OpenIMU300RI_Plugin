#include <imu.h>
using namespace std;

typedef enum{
  NONE                  = 0,
  REQUEST_PACKET        = 1,
  CONFIGURATION_PACKET  = 2,
  REQ_CONFIG_PACKET     = 3,
  DATA_PACKET           = 4,
  REQ_DATA_PACKET       = 5,
}PACKET_TYPE_t;

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
  // Add New Messages here
  MAX_PGN,
}imuMessages;

// Note 1: Keeping the PS parameters uptop because the plugin parses user parameter
// string in this order. Allowing to look for PS changes in user parameter
// string and change PS values for fields before actully configuring the field.
// Eg. User want to change the PS for Packet rate and configure the Packet rate
// to some value, this mechanism allows to first change the PS number for
// Packet Rate and then uses new PS number to configure the Packet Rate field.
// Note 2: Order of this enum is important be and must match with
// vector<string>paraNames in openIMU300_plugin.cpp
typedef enum{
  PARAM_RESET_ALGO_PS,
  PARAM_SET_PACKET_RATE_PS,
  PARAM_SET_PACKET_TYPE_PS,
  PARAM_SET_FILTER_CUTOFF_PS,
  PARAM_SET_ORIENTATION_PS,
  PARAM_PACKET_RATE,
  PARAM_PACKET_TYPE,
  PARAM_ORIENTATION,
  PARAM_RATE_LPF,
  PARAM_ACCEL_LPF,
  PARAM_RESET_ALGO,
  //Add New Params here
  PARAM_MAX_PARAMS,
} IMUPARAM_t;

// angular rate data payload format
typedef struct {
    uint16_t roll_rate;                             // roll  rate
    uint16_t pitch_rate;                            // pitch rate
    uint16_t yaw_rate;                              // yaw   rate
    uint8_t  pitch_merit          :       2;        // pitch rate merit
    uint8_t  roll_merit           :       2;        // roll  rate merit
    uint8_t  yaw_merit            :       2;        // yaw  rate merit
    uint8_t  rsvd                 :       2;        // rsvd
    uint8_t  measurement_latency;                   // latency
} angularRate;

// accleration data payload format
typedef struct {
    uint16_t   acceleration_x;                      // x-axis acceleration
    uint16_t   acceleration_y;                      // y-axis acceleration
    uint16_t   acceleration_z;                      // z-axis acceleration
    uint8_t    lateral_merit        :       2;      // laterar acc merit
    uint8_t    longitudinal_merit   :       2;      // longitudinal merit
    uint8_t    vertical_merit       :       2;      // vertical merit
    uint8_t    transmit_rate        :       2;      // repetition rate
    uint8_t    rsvd;
} accelSensor;

// accleration data payload format
typedef struct {
    uint16_t   mag_x;                        // x-axis mag data
    uint16_t   mag_y;                        // y-axis mag data
    uint16_t   mag_z;                        // z-axis mag data
    uint16_t   unuzed;
} magSensor;

// slope sensor data payload format
typedef struct {
    uint64_t pitch                :       24;       // pitch
    uint64_t roll                 :       24;       // roll
    uint64_t pitch_compensation   :       2;        // pitch compensation
    uint64_t pitch_merit          :       2;        // pitch merit
    uint64_t roll_compensation    :       2;        // roll compensation
    uint64_t roll_merit           :       2;        // roll merit
    uint64_t measure_latency      :       8;        // latency
} slopeSensor;

struct pgn{
  PACKET_TYPE_t type;
  uint8_t PF;
  uint8_t PS;
  pgn(){type = NONE, PF = 0; PS = 0;}
  pgn(PACKET_TYPE_t type, uint8_t pf, uint8_t ps): type(type), PF(pf), PS(ps){}
};

typedef struct{
  uint16_t packetRate;
  uint16_t packetType;
  uint16_t orientation;
  uint16_t rateLPF;
  uint16_t accelLPF;
  uint8_t resetAlgo;
} imuParameters_t;

class OpenIMU300 : public IMU
{
  public:
    OpenIMU300();

    OpenIMU300(uint8_t srcAddr, uint8_t destAddr = 0x80);

    virtual ~OpenIMU300() override;

    virtual bool init(string paramsString, vector<dwCANMessage> &configMessages) override;

    virtual bool isValidMessage(uint32_t message_id) override;

    virtual bool parseDataPacket(dwCANMessage packet, dwIMUFrame *IMUframe) override;

  private:
    imuMessages findDataPacket(uint8_t pf, uint8_t ps);

    void getBankOfPSPacket(uint8_t bank, uint8_t *reg, dwCANMessage *packet);

    void getConfigPacket(IMUPARAM_t param, uint16_t paramVal, dwCANMessage *packet);

    void getPacketIdentifiers(uint32_t id, uint8_t *pf, uint8_t *ps);

    bool getParameterVal(string searchString, string userString, uint16_t* value);

    bool getParams(string paramsString, vector<dwCANMessage> &configMessages);

    bool isValidBankOfPSPacket(uint16_t value);

    template<typename T>
    bool isValidConfigRequest(const std::vector<T> validValues, const T value);

    void printPSList();

    uint8_t                       SRCAddress;
    uint8_t                       ECUAddress;
    imuParameters_t               imuParameter;
    map<uint8_t,vector<uint8_t>>  PGNMap;      //map<PF,Vector<PS,PS,PS>>
};
