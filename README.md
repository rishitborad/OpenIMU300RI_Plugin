# OpenIMU300RI_Plugin



#Parameters:

Append below parameters to the parameter string when running the plugin. The plugin will prepare CAN packet for each parameter in the parameter list and send it to the IMU. User can also send no parameter, in that case, the plugin will not send any configuration packet to the IMU and run on default IMU configuration settings.

IMU should only be configured using the plugin. Configuring IMU outside this plugin will not work. Plugin doesn't provide support to save parameters permanently. Plugin parameters will reset to default each time the plugin is started/restarted. Users who want to run the IMU with custom configuration are advised to send configuration parameter each time the plugin is started.

Plugin only supports decimal parameter values. Also plugin will not start if it detects a wrong parameter values. Each parameter value must be valid. See parameter table for valid parameter name and values.

example usage:

    `--params=decoder-path=../libopenimu_plugin.so,can-proto=can.socket,device=slcan0,packetRate=1,packetType=2,orientation=0,rateLPF=5,accelLPF=25,resetAlgo=1,resetAlgoPS=64,setPacketRatePS=65,setPacketTypePS=66,setFilterCutoffPS=67,setOrientationPS=68`
    `--params=decoder-path=../libopenimu_plugin.so,can-proto=can.socket,device=slcan0,packetRate=1,packetType=2,orientation=0`
    `--params=decoder-path=../libopenimu_plugin.so,can-proto=can.socket,device=slcan0,packetRate=1`

Parameter Table

|Parameter Name         |Description                                 |Valid values (Decimal only)
------------------------------------------------------------------------------------------------
|packetRate=            |Packet Rate                                 |0,1,2,4,5,10,20,25,50
|packetType=            |Packet Type                                 |0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
|orientation=            |Orientation                                 |0x0000, 0x0009, 0x0023, 0x002A,0x0041, 0x0048, 0x0062, 0x006B,0x0085, 0x008C, 0x0092, 0x009B
|rateLPF=                |Rate Sensor Cutoff Frequency                |0,2,5,10,25,40,50
|accelLPF=               |Accel Sensor Cutoff Frequency               |0,2,5,10,25,40,50
|resetAlgo=              |Reset Algorithm                             |1
|resetAlgoPS=            |Bank of PS for Reset Algorithm              |Range[0x40,0x80)
|setPacketRatePS=        |Bank of PS for Packet Rate                  |Range[0x40,0x80)
|setPacketTypePS=        |Bank of PS for Packet Type                  |Range[0x40,0x80)
|setFilterCutoffPS=      |Bank of PS for Digital Filter Cutoff Freq   |Range[0x40,0x80)
|setOrientationPS=       |Bank of PS for Orientation                  |Range[0x40,0x80)


Note: Detailed information on each valid values can be found at https://openimu.readthedocs.io/en/latest/software/CAN/CAN_J1939_CAN_Messages.html
