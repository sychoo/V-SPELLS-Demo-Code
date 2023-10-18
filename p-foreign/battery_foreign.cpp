#include "Prt.h"

PRT_VALUE *P_RetrieveBattery_IMPL(PRT_MACHINEINST *context, PRT_VALUE ***argRefs)
{
     mavlink_message_t message;
     mavlink_command_long_t battery = {0};
     battery.target_system = 1;
     battery.target_component = 0;
     battery.command = MAV_CMD_REQUEST_MESSAGE;
     battery.param1 = MAVLINK_MSG_ID_BATTERY_STATUS;

     mavlink_msg_command_long_encode(1, 255, &message, &battery);

     uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
     const int len = mavlink_msg_to_send_buffer(buffer, &message);

     int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr *)src_addr, src_addr_len);
     if (ret != len)
     {
          printf("sendto error: %s\n", strerror(errno));
     }
     else
     {
          printf("Drone is Taking Off\n");
     }

     message = receive_message();
     if (message.gid == MAVLINK_MSG_ID_BATTERY_STATUS)
     {
          mavlink_battery_status_t batteryinfo;
          mavlink_msg_battery_status_decode(message, &batteryinfo);
          return PrtMkFloatValue((PRT_FLOAT)batteryinfo.battery_remaining);
     }
     return PrtMkFloatValue((PRT_FLOAT)-1.0f);
}

PRT_VALUE *P_EmergencyLand_IMPL(PRT_MACHINEINST *context, PRT_VALUE ***argRefs)
{
     mavlink_message_t message;
     mavlink_command_long_t set_mode = {0};

     set_mode.target_system = 1;
     set_mode.target_component = 0;
     set_mode.command = MAV_CMD_DO_SET_MODE;
     set_mode.confirmation = true;
     set_mode.param1 = 1;
     set_mode.param2 = 9;

     mavlink_msg_command_long_encode(1, 0, &message, &set_mode);

     uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
     const int len = mavlink_msg_to_send_buffer(buffer, &message);

     int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr *)src_addr, src_addr_len);
     if (ret != len)
     {
          return PrtMkBoolValue(PRT_FALSE);
     }
     else
     {
          printf("Emergency Landing\n");
     }
     return PrtMkBoolValue(PRT_TRUE);
}