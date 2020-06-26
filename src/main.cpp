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
#include <openimu300_plugin.h>
#include <map>
#include <unistd.h>
using namespace std;
namespace dw
{
namespace plugins
{
namespace imu
{

#define SRC_ADDRESS       (0x00)
#define DEST_ADDRESS      (0x80)
static OpenIMU300 instance(SRC_ADDRESS, DEST_ADDRESS);

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

const size_t SAMPLE_BUFFER_POOL_SIZE = 5;

class AceinnaIMUSensor
{
public:
    AceinnaIMUSensor(dwContextHandle_t ctx, dwSensorHandle_t canSensor, size_t slotSize)
        : m_ctx(ctx)
        , m_sal(nullptr)
        , m_canSensor(canSensor)
        , m_virtualSensorFlag(true)
        , m_buffer(sizeof(dwCANMessage))
        , m_slot(slotSize)
        , imu(&instance)
    {
    }

    ~AceinnaIMUSensor() = default;

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

        // Initialize IMU and get list of paramater strings supported and set parameter struct to default
        if(!imu->init(paramsString, configMessages))
        {
          return DW_FAILURE;
        }

        for(size_t i = 0; i < configMessages.size(); i++)
        {
          printf("configMessages[%lu].id = %X\r\n", i, configMessages[i].id);
        }

        return DW_SUCCESS;
    }

    dwStatus startSensor()
    {
        dwStatus status;
        if (!isVirtualSensor())
        {
          status = dwSensor_start(m_canSensor);

          if(status != DW_SUCCESS)
            return status;

          dwCANMessage resetMessage;
          imu->getSensorResetMessage(&resetMessage);
          if(dwSensorCAN_sendMessage(&resetMessage, 100000, m_canSensor) != DW_SUCCESS){
            return DW_FAILURE;
          }
          printf("restarting IMU\r\n");
          sleep(10);

          // Configure IMU to parameters received in parameter string
          // for each paramater in map<parameterName,paramVal> send a message to
          // the IMU to configure according to the plugin user request
          for(size_t i = 0; i < configMessages.size(); i++)
          {
            printf("configMessages[%lu].id = %X, %X %X %X %X %X %X %X %X\r\n", i, configMessages[i].id, configMessages[i].data[0], configMessages[i].data[1], configMessages[i].data[2], configMessages[i].data[3], configMessages[i].data[4], configMessages[i].data[5], configMessages[i].data[6], configMessages[i].data[7]);
            if(dwSensorCAN_sendMessage(&configMessages[i], 100000, m_canSensor) != DW_SUCCESS)
              return DW_FAILURE;
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
          if(imu->isValidMessage(result->id))
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
        //cout << "Pushing Data\r\n";
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

        if(!imu->parseDataPacket(*reference, frame))
        {
          m_buffer.dequeue();
          return DW_FAILURE;
        }

        m_buffer.dequeue();
        return DW_SUCCESS;
    }

    static std::vector<std::unique_ptr<dw::plugins::imu::AceinnaIMUSensor>> g_sensorContext;

private:
    inline bool isVirtualSensor()
    {
        return m_virtualSensorFlag;
    }

    dwContextHandle_t m_ctx      = nullptr;
    dwSALHandle_t m_sal          = nullptr;
    dwSensorHandle_t m_canSensor = nullptr;
    bool m_virtualSensorFlag;

    dw::plugin::common::ByteQueue m_buffer;
    dw::plugins::common::BufferPool<dwCANMessage> m_slot;

    IMU                   *imu;    // Pointer to IMU abstract class
    vector<dwCANMessage>  configMessages;
};
} // namespace imu
} // namespace plugins
} // namespace dw

std::vector<std::unique_ptr<dw::plugins::imu::AceinnaIMUSensor>> dw::plugins::imu::AceinnaIMUSensor::g_sensorContext;

//#######################################################################################
static bool checkValid(dw::plugins::imu::AceinnaIMUSensor* sensor)
{
    for (auto& i : dw::plugins::imu::AceinnaIMUSensor::g_sensorContext)
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
    auto sensorContext = new dw::plugins::imu::AceinnaIMUSensor(ctx, DW_NULL_HANDLE, slotSize);

    dw::plugins::imu::AceinnaIMUSensor::g_sensorContext.push_back(std::unique_ptr<dw::plugins::imu::AceinnaIMUSensor>(sensorContext));
    *sensor = dw::plugins::imu::AceinnaIMUSensor::g_sensorContext.back().get();

    // Populate sensor properties
    properties->packetSize = sizeof(dwCANMessage);

    return DW_SUCCESS;
}

//#######################################################################################
dwStatus _dwSensorPlugin_createSensor(const char* params, dwSALHandle_t sal, dwSensorPluginSensorHandle_t sensor)
{

    auto sensorContext = reinterpret_cast<dw::plugins::imu::AceinnaIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->createSensor(sal, params);
}

//#######################################################################################
dwStatus _dwSensorPlugin_start(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::AceinnaIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->startSensor();
}

//#######################################################################################
dwStatus _dwSensorPlugin_release(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::AceinnaIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    for (auto iter =
                    dw::plugins::imu::AceinnaIMUSensor::g_sensorContext.begin();
                    iter != dw::plugins::imu::AceinnaIMUSensor::g_sensorContext.end();
                    ++iter)
    {
        if ((*iter).get() == sensor)
        {
            sensorContext->stopSensor();
            sensorContext->releaseSensor();
            dw::plugins::imu::AceinnaIMUSensor::g_sensorContext.erase(iter);
            return DW_SUCCESS;
        }
    }
    return DW_FAILURE;
}

//#######################################################################################
dwStatus _dwSensorPlugin_stop(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::AceinnaIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->stopSensor();
}

//#######################################################################################
dwStatus _dwSensorPlugin_reset(dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::AceinnaIMUSensor*>(sensor);
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
    auto sensorContext = reinterpret_cast<dw::plugins::imu::AceinnaIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->readRawData(data, size, timeout_us);
}

//#######################################################################################
dwStatus _dwSensorPlugin_returnRawData(const uint8_t* data, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::AceinnaIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->returnRawData(data);
}

//#######################################################################################
dwStatus _dwSensorPlugin_pushData(size_t* lenPushed, const uint8_t* data, const size_t size, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::AceinnaIMUSensor*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->pushData(data, size, lenPushed);
}

//#######################################################################################
dwStatus _dwSensorIMUPlugin_parseDataBuffer(dwIMUFrame* frame, size_t* consumed, dwSensorPluginSensorHandle_t sensor)
{
    auto sensorContext = reinterpret_cast<dw::plugins::imu::AceinnaIMUSensor*>(sensor);
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
