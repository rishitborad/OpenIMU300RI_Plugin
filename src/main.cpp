/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2019 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <dw/sensors/plugins/imu/IMUPlugin.h>
#include <dw/sensors/canbus/CAN.h>
#include <BufferPool.hpp>
#include <ByteQueue.hpp>
#include <iostream>
#include <openimu300.h>
#include <imu_messaging.h>

namespace dw
{
namespace plugins
{
namespace imu
{

  static OpenIMU300 instance;

// Structure defining sample CAN acceleration
typedef struct
{
    int16_t accelLat;
    int16_t accelLong;
    int16_t accelVert;
} SampleCANReportAccel;

// Structure defining sample CAN gyro
typedef struct
{
    int16_t gyroRoll;
    int16_t gyroYaw;
} SampleCANReportGyro;

const uint32_t SAMPLE_CAN_REPORT_ACCEL = 0x06B;
const uint32_t SAMPLE_CAN_REPORT_GYRO  = 0x06C;
const uint32_t OPENIMU_AR  = 0x0CF02A80;
const size_t SAMPLE_BUFFER_POOL_SIZE = 5;
#if 0
static bool getParameterVal(std::string paramsString, std::string searchString, uint16_t* value)
{
  std::string param;
  size_t pos = paramsString.find(searchString);
  *value = 0;
  if (pos != std::string::npos)
  {
      param   = paramsString.substr(pos);
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

static void initParams(std::string paramsString, imuParameters_t *params)
{
  for(unsigned int i = 0; i < paramNames.size(); i++)
  {
    uint16_t val = 0;
    if(getParameterVal(paramsString, paramNames[i], &val))
    {
      switch(static_cast<IMU_PARAMS_t>(i))
      {
        case IMU_PARAMS_t::PACKET_TYPE: params->packetType  = val;  break;
        case IMU_PARAMS_t::PACKET_RATE: params->packetRate  = val;  break;
        case IMU_PARAMS_t::ORIENTATION: params->orientation = val;  break;
        case IMU_PARAMS_t::RATE_LPF:    params->rateLPF     = val;  break;
        case IMU_PARAMS_t::ACCEL_LPF:   params->accelLPF    = val;  break;
        default:
          break;
      }
    }
  }
}
#endif
class SampleIMUSensor
{
public:
    SampleIMUSensor(dwContextHandle_t ctx, dwSensorHandle_t canSensor, size_t slotSize)
        : m_ctx(ctx)
        , m_sal(nullptr)
        , m_canSensor(canSensor)
        , m_virtualSensorFlag(true)
        , m_buffer(sizeof(dwCANMessage))
        , m_slot(slotSize)
        //, imuParams(defaultParams)
        , imu300(&instance)
    {
    }

    ~SampleIMUSensor() = default;

    dwStatus createSensor(dwSALHandle_t sal, const char* params)
    {
        m_sal = sal;
        m_virtualSensorFlag = false;

        std::string paramsString = params;
        std::string searchString = "can-proto=";
        size_t pos               = paramsString.find(searchString);

        if (pos == std::string::npos)
        {
            std::cerr << "createSensor: Protocol not specified\n";
            return DW_FAILURE;
        }

        std::string protocolString = paramsString.substr(pos + searchString.length());
        pos                        = protocolString.find_first_of(",");
        protocolString             = protocolString.substr(0, pos);

        // create CAN bus interface
        dwSensorParams parameters{};
        parameters.parameters = params;
        parameters.protocol   = protocolString.c_str();

        if (dwSAL_createSensor(&m_canSensor, parameters, m_sal) != DW_SUCCESS)
        {
            std::cerr << "createSensor: Cannot create sensor "
                      << parameters.protocol << " with " << parameters.parameters << std::endl;

            return DW_FAILURE;
        }

        // Setup IMU parameters
        //initParams(paramsString, &imuParams);

        //std::cout<<"PARAM LIST: "<<imuParams.packetRate<<","<<imuParams.packetType<<","<<imuParams.orientation<<","<<imuParams.rateLPF<<","<<imuParams.accelLPF<<"\r\n";

        return DW_SUCCESS;
    }

    dwStatus startSensor()
    {
        dwStatus status;
        if (!isVirtualSensor())
        {
          status = dwSensor_start(m_canSensor);

          std::cout << "PACKET RATE\r\n";
          imu300->isValidMessage(0);
          // Configure Sensor
          if(status == DW_SUCCESS)
          {
          #if 0
            uint8_t payload[2] = {128,50};
            std::cout << "PACKET RATE\r\n";

            if(imuParams.packetRate != 0)
            {
              dwCANMessage sendPacketRate;
              sendPacketRate.id           = 0x18FF5500;
              sendPacketRate.size         = 2;
              sendPacketRate.timestamp_us = 0;
              //sendPacketRate.data = payload;
              memcpy(sendPacketRate.data, payload, sendPacketRate.size);

              dwStatus status = dwSensorCAN_sendMessage(&sendPacketRate, 100000, m_canSensor);

              std::cout << status <<"PACKET RATE SENT\r\n";
            }
          #endif
          }
          else{
              return status;
          }
        }
        return DW_SUCCESS;
    }

    dwStatus releaseSensor()
    {
        if (!isVirtualSensor())
            return dwSAL_releaseSensor(m_canSensor);

        return DW_SUCCESS;
    }

    dwStatus stopSensor()
    {
        if (!isVirtualSensor())
            return dwSensor_stop(m_canSensor);

        return DW_SUCCESS;
    }

    dwStatus resetSensor()
    {
        m_buffer.clear();

        if (!isVirtualSensor())
            return dwSensor_reset(m_canSensor);

        return DW_SUCCESS;
    }

    dwStatus readRawData(const uint8_t** data, size_t* size, dwTime_t timeout_us)
    {
        dwCANMessage* result = nullptr;
        bool ok              = m_slot.get(result);
        if (!ok)
        {
            std::cerr << "readRawData: Read raw data, slot not empty\n";
            return DW_BUFFER_FULL;
        }

        while (dwSensorCAN_readMessage(result, timeout_us, (m_canSensor)) == DW_SUCCESS)
        {
            // Filter invalid messages
            if (result->id == SAMPLE_CAN_REPORT_ACCEL || result->id == SAMPLE_CAN_REPORT_GYRO || result->id == OPENIMU_AR)
            {
                break;
            }
        }

        *data = reinterpret_cast<uint8_t*>(result);
        *size = sizeof(dwCANMessage);
        return DW_SUCCESS;
    }

    dwStatus returnRawData(const uint8_t* data)
    {
        if (data == nullptr)
        {
            return DW_INVALID_HANDLE;
        }

        bool ok = m_slot.put(const_cast<dwCANMessage*>(reinterpret_cast<const dwCANMessage*>(data)));
        if (!ok)
        {
            std::cerr << "returnRawData: IMUPlugin return raw data, invalid data pointer" << std::endl;
            return DW_INVALID_ARGUMENT;
        }
        data = nullptr;

        return DW_SUCCESS;
    }

    dwStatus pushData(const uint8_t* data, const size_t size, size_t* lenPushed)
    {
        m_buffer.enqueue(data, size);
        *lenPushed = size;
        return DW_SUCCESS;
    }

    dwStatus parseData(dwIMUFrame* frame, size_t* consumed)
    {
        const dwCANMessage* reference;

        if (!m_buffer.peek(reinterpret_cast<const uint8_t**>(&reference)))
        {
            return DW_NOT_AVAILABLE;
        }

        if (consumed)
            *consumed = sizeof(dwCANMessage);

        *frame              = {};
        frame->timestamp_us = (*reference).timestamp_us;

        switch ((*reference).id)
        {
          case SAMPLE_CAN_REPORT_ACCEL:
          {
              auto ptr = reinterpret_cast<const SampleCANReportAccel*>((*reference).data);
              // All fields have range: -327.68 to 327.67, LSB: 0.01 m/s^2, thus
              // multiply by 0.01f to convert to m/s^2
              frame->acceleration[0] = static_cast<float32_t>(ptr->accelLong) * 0.01f;
              frame->acceleration[1] = static_cast<float32_t>(ptr->accelLat) * 0.01f;
              frame->acceleration[2] = static_cast<float32_t>(ptr->accelVert) * 0.01f;
              frame->flags |= DW_IMU_ACCELERATION_X | DW_IMU_ACCELERATION_Y | DW_IMU_ACCELERATION_Z;
              break;
          }
          case SAMPLE_CAN_REPORT_GYRO:
          {
              auto ptr = reinterpret_cast<const SampleCANReportGyro*>((*reference).data);
              // All fields have range: -6.5536 to 6.5534, LSB: 0.0002 rad/s, thus
              // multiply by 0.0002 to convert to rad/s
              frame->turnrate[0] = static_cast<float32_t>(ptr->gyroRoll) * 0.0002f;
              frame->turnrate[1] = 0; // XXX: need to extend DataSpeed protocol for this
              frame->turnrate[2] = static_cast<float32_t>(ptr->gyroYaw) * 0.0002f;
              frame->flags |= DW_IMU_ROLL_RATE | DW_IMU_YAW_RATE;
              break;
          }
          case OPENIMU_AR:
          {
              auto ptr = reinterpret_cast<const angularRate*>((*reference).data);
              // All fields have range: -6.5536 to 6.5534, LSB: 0.0002 rad/s, thus
              // multiply by 0.0002 to convert to rad/s
              frame->turnrate[0] = static_cast<float32_t>(ptr->roll_rate) * 0.0002f;
              frame->turnrate[1] = 0; // XXX: need to extend DataSpeed protocol for this
              frame->turnrate[2] = static_cast<float32_t>(ptr->pitch_rate) * 0.0002f;
              frame->flags |= DW_IMU_ROLL_RATE | DW_IMU_YAW_RATE;
              break;
          }
          default:
              m_buffer.dequeue();
              return DW_FAILURE;
        }
        m_buffer.dequeue();
        return DW_SUCCESS;
    }

    static std::vector<std::unique_ptr<dw::plugins::imu::SampleIMUSensor>> g_sensorContext;

private:
    inline bool isVirtualSensor()
    {
        return m_virtualSensorFlag;
    }

#if 0
    void setupIMU()
    {
      dwCANMessage sendPacketRate;
      for(unsigned int i = 0; i < IMU_PARAMS_t::MAX_IMU_PARAMS; i++)
      {
        switch(static_cast<IMU_PARAMS_t>(i))
        {
          case IMU_PARAMS_t::PACKET_TYPE:
            if(imuParams.packetType != defaultParams.packetType)
            {
                uint8_t payload[2] = {128,0};
                payload[1] = static_cast<uint8_t>(imuParams.packetType);
                sendPacketRate.id           = 0x18FF5500;
                sendPacketRate.size         = 2;
                sendPacketRate.timestamp_us = 0;
                //sendPacketRate.data = payload;
                memcpy(sendPacketRate.data, payload, sendPacketRate.size);
            }
            break;
          case IMU_PARAMS_t::PACKET_RATE:
            if(imuParams.packetRate!= defaultParams.packetRate)
            {
                uint8_t payload[2] = {128,0};
                payload[1] = static_cast<uint8_t>(imuParams.packetType);
                sendPacketRate.id           = 0x18FF5500;
                sendPacketRate.size         = 2;
                sendPacketRate.timestamp_us = 0;
                //sendPacketRate.data = payload;
                memcpy(sendPacketRate.data, payload, sendPacketRate.size);
            }
            break;
          case IMU_PARAMS_t::ORIENTATION:
            if(imuParams.orientation != defaultParams.orientation)
            {
                uint8_t payload[2] = {128,0};
                payload[1] = static_cast<uint8_t>(imuParams.packetType);
                sendPacketRate.id           = 0x18FF5500;
                sendPacketRate.size         = 2;
                sendPacketRate.timestamp_us = 0;
                //sendPacketRate.data = payload;
                memcpy(sendPacketRate.data, payload, sendPacketRate.size);
            }
            break;
          case IMU_PARAMS_t::RATE_LPF:
            if(imuParams.rateLPF != defaultParams.rateLPF)
            {
                uint8_t payload[2] = {128,0};
                payload[1] = static_cast<uint8_t>(imuParams.packetType);
                sendPacketRate.id           = 0x18FF5500;
                sendPacketRate.size         = 2;
                sendPacketRate.timestamp_us = 0;
                //sendPacketRate.data = payload;
                memcpy(sendPacketRate.data, payload, sendPacketRate.size);
            }
            break;
          case IMU_PARAMS_t::ACCEL_LPF:
            if(imuParams.accelLPF != defaultParams.accelLPF)
            {
                uint8_t payload[2] = {128,0};
                payload[1] = static_cast<uint8_t>(imuParams.packetType);
                sendPacketRate.id           = 0x18FF5500;
                sendPacketRate.size         = 2;
                sendPacketRate.timestamp_us = 0;
                //sendPacketRate.data = payload;
                memcpy(sendPacketRate.data, payload, sendPacketRate.size);
            }
            break;
          default:
            break;
        }
        dwStatus status = dwSensorCAN_sendMessage(&sendPacketRate, 100000, m_canSensor);
      }
    }
  #endif
    dwContextHandle_t m_ctx      = nullptr;
    dwSALHandle_t m_sal          = nullptr;
    dwSensorHandle_t m_canSensor = nullptr;
    bool m_virtualSensorFlag;

    dw::plugin::common::ByteQueue m_buffer;
    dw::plugins::common::BufferPool<dwCANMessage> m_slot;

    //imuParameters_t imuParams;

    IMUMessaging *imu300;

};
} // namespace imu
} // namespace plugins
} // namespace dw

std::vector<std::unique_ptr<dw::plugins::imu::SampleIMUSensor>> dw::plugins::imu::SampleIMUSensor::g_sensorContext;

//#######################################################################################
static bool checkValid(dw::plugins::imu::SampleIMUSensor* sensor)
{
    for (auto& i : dw::plugins::imu::SampleIMUSensor::g_sensorContext)
    {
        if (i.get() == sensor)
            return true;
    }
    return false;
}

// exported functions
extern "C" {

//#######################################################################################
dwStatus _dwSensorPlugin_createHandle(dwSensorPluginSensorHandle_t* sensor, dwSensorPluginProperties* properties,
                                      const char *, dwContextHandle_t ctx)
{
    if (!sensor)
        return DW_INVALID_ARGUMENT;

    size_t slotSize    = dw::plugins::imu::SAMPLE_BUFFER_POOL_SIZE; // Size of memory pool to read raw data from the sensor
    auto sensorContext = new dw::plugins::imu::SampleIMUSensor(ctx, DW_NULL_HANDLE, slotSize);

    dw::plugins::imu::SampleIMUSensor::g_sensorContext.push_back(std::unique_ptr<dw::plugins::imu::SampleIMUSensor>(sensorContext));
    *sensor = dw::plugins::imu::SampleIMUSensor::g_sensorContext.back().get();

    // Populate sensor properties
    properties->packetSize = sizeof(dwCANMessage);

    return DW_SUCCESS;
}

//#######################################################################################
dwStatus _dwSensorPlugin_createSensor(const char* params, dwSALHandle_t sal, dwSensorPluginSensorHandle_t sensor)
{

    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->createSensor(sal, params);
}

//#######################################################################################
dwStatus _dwSensorPlugin_start(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->startSensor();
}

//#######################################################################################
dwStatus _dwSensorPlugin_release(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    for (auto iter =
                    dw::plugins::imu::SampleIMUSensor::g_sensorContext.begin();
                    iter != dw::plugins::imu::SampleIMUSensor::g_sensorContext.end();
                    ++iter)
    {
        if ((*iter).get() == sensor)
        {
            sensorContext->stopSensor();
            sensorContext->releaseSensor();
            dw::plugins::imu::SampleIMUSensor::g_sensorContext.erase(iter);
            return DW_SUCCESS;
        }
    }
    return DW_FAILURE;
}

//#######################################################################################
dwStatus _dwSensorPlugin_stop(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->stopSensor();
}

//#######################################################################################
dwStatus _dwSensorPlugin_reset(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->resetSensor();
}

//#######################################################################################
dwStatus _dwSensorPlugin_readRawData(const uint8_t** data, size_t* size, dwTime_t* /*timestamp*/,
                                     dwTime_t timeout_us, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->readRawData(data, size, timeout_us);
}

//#######################################################################################
dwStatus _dwSensorPlugin_returnRawData(const uint8_t* data, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->returnRawData(data);
}

//#######################################################################################
dwStatus _dwSensorPlugin_pushData(size_t* lenPushed, const uint8_t* data, const size_t size, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->pushData(data, size, lenPushed);
}

//#######################################################################################
dwStatus _dwSensorIMUPlugin_parseDataBuffer(dwIMUFrame* frame, size_t* consumed, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::SampleIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->parseData(frame, consumed);
}

//#######################################################################################
dwStatus dwSensorIMUPlugin_getFunctionTable(dwSensorIMUPluginFunctionTable* functions)
{
    if (functions == nullptr)
        return DW_INVALID_ARGUMENT;

    functions->common.createHandle  = _dwSensorPlugin_createHandle;
    functions->common.createSensor  = _dwSensorPlugin_createSensor;
    functions->common.release       = _dwSensorPlugin_release;
    functions->common.start         = _dwSensorPlugin_start;
    functions->common.stop          = _dwSensorPlugin_stop;
    functions->common.reset         = _dwSensorPlugin_reset;
    functions->common.readRawData   = _dwSensorPlugin_readRawData;
    functions->common.returnRawData = _dwSensorPlugin_returnRawData;
    functions->common.pushData      = _dwSensorPlugin_pushData;
    functions->parseDataBuffer      = _dwSensorIMUPlugin_parseDataBuffer;

    return DW_SUCCESS;
}

} // extern "C"
