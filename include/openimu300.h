#include <string>
#include <vector>
using namespace std;

namespace dw
{
namespace plugins
{
namespace imu
{

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
/*
typedef enum {
  PACKET_TYPE = 0,
  PACKET_RATE = 1,
  ORIENTATION = 2,
  RATE_LPF = 3,
  ACCEL_LPF = 4,
  MAX_IMU_PARAMS = 5
}IMU_PARAMS_t;
*/
typedef struct{
  uint16_t packetRate;
  uint16_t packetType;
  uint16_t orientation;
  uint16_t rateLPF;
  uint16_t accelLPF;
} imuParameters_t;

const imuParameters_t defaultParams = {
          .packetRate   = 1,
          .packetType   = 1,
          .orientation  = 0,
          .rateLPF      = 0x0000,
          .accelLPF     = 0,
};

const vector<string> paramNames = {"packet-rate=","packet-type=","orientation=","rateLPF=","accelLPF="};

} // namespace imu
} // namespace plugins
} // namespace dw
