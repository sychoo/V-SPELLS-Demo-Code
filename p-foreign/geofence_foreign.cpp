#include "Prt.h"

PRT_VALUE *P_DefineGeofence_IMPL(PRT_MACHINEINST *context, PRT_VALUE ***argRefs)
{
	mavlink_mission_item_int_t mission = {0};
	mission.target_system = 1;
	mission.target_component = 0;
	mission.command = MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
    mission.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    mission.current = 2;
    mission.autocontinue = 0;
    mission.mission_type = MAV_MISSION_TYPE_FENCE;

	mission.param1 = radius;
    mission.param2 = 0;
 	mission.x = (int) (-35.36277334 * 10000000.0);
    mission.y = (int) (149.16536671 * 10000000.0);	 	

	mavlink_message_t msg;
    mavlink_msg_mission_item_int_encode(1, 255, &msg, &mission);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &msg);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("GeoFence is Defined for the Drone\n");
    }
    return PrtMkBoolValue(PRT_TRUE);
}

PRT_VALUE *P_ReturnToHome_IMPL(PRT_MACHINEINST *context, PRT_VALUE ***argRefs)
{
     mavlink_message_t message;
     mavlink_command_long_t set_mode = {0};

     set_mode.target_system = 1;
     set_mode.target_component = 0;
     set_mode.command = MAV_CMD_DO_SET_MODE;
     set_mode.confirmation = true;
     set_mode.param1 = 1;
     set_mode.param2 = 6;

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

PRT_VALUE *P_RequestDroneTelemetryLat_IMPL(PRT_MACHINEINST *context, PRT_VALUE ***argRefs)
{
    // Create a request message
    mavlink_message_t msg;
    mavlink_msg_gps_raw_int_pack(system->sysid, system->compid, &msg, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    // Send the request message
    mavlink_msg_to_send_buffer(g_send_buffer, &msg);
    uint16_t len = mavlink_msg_to_send_buffer(g_send_buffer, &msg);
    serialPortSend(g_serial_port, g_send_buffer, len); // Replace with your send function

    return PrtMkFloatValue((PRT_FLOAT) drone_lat);
}


PRT_VALUE *P_RequestDroneTelemetryLon_IMPL(PRT_MACHINEINST *context, PRT_VALUE ***argRefs)
{
    // Create a request message
    mavlink_message_t msg;
    mavlink_msg_gps_raw_int_pack(system->sysid, system->compid, &msg, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    // Send the request message
    mavlink_msg_to_send_buffer(g_send_buffer, &msg);
    uint16_t len = mavlink_msg_to_send_buffer(g_send_buffer, &msg);
    serialPortSend(g_serial_port, g_send_buffer, len); // Replace with your send function

    return PrtMkFloatValue((PRT_FLOAT) drone_lon);
}

PRT_VALUE *P_DistanceToHome_IMPL(PRT_MACHINEINST *context, PRT_VALUE ***argRefs)
{
    // Convert latitude and longitude from degrees to radians
    lat1 = degreesToRadians(lat1);
    lon1 = degreesToRadians(lon1);
    lat2 = degreesToRadians(lat2);
    lon2 = degreesToRadians(lon2);

    // Haversine formula
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance = RADIUS_OF_EARTH * c;

    return PrtMkFloatValue((PRT_FLOAT) distance);
}
