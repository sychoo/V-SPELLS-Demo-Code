#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <time.h>
#include <unistd.h>
#include <mavlink/ardupilotmega/mavlink.h>

void retrieve_battery(int socket_fd, const struct sockaddr_in *src_addr, socklen_t src_addr_len)
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
}

void takeoff(int socket_fd, const struct sockaddr_in *src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;

    mavlink_command_long_t takeoff = {0};
    takeoff.target_system = 1;
    takeoff.target_component = 0;
    takeoff.command = MAV_CMD_NAV_TAKEOFF;
    takeoff.confirmation = true;
    takeoff.param7 = 20;

    mavlink_msg_command_long_encode(1, 255, &message, &takeoff);

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
}

void define_geofence(int socket_fd, const struct sockaddr_in *src_addr, socklen_t src_addr_len)
{
    mavlink_mission_item_int_t mission = {0};
    mission.target_system = 1;
    mission.target_component = 0;
    mission.command = MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION; //; 	//213 (MAV_CMD_DO_SET_POSITION_YAW_THRUST)
    mission.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;    // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    mission.current = 2;
    mission.autocontinue = 0;
    mission.mission_type = MAV_MISSION_TYPE_FENCE;

    mission.param1 = 200;
    mission.param2 = 0;
    mission.x = (int)(-35.36277334 * 10000000.0);
    mission.y = (int)(149.16536671 * 10000000.0);

    mavlink_message_t msg;
    mavlink_msg_mission_item_int_encode(1, 255, &msg, &mission);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &msg);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr *)src_addr, src_addr_len);
    if (ret != len)
    {
        printf("sendto error: %s\n", strerror(errno));
    }
    else
    {
        printf("GeoFence is Defined for the Drone\n");
    }
}
