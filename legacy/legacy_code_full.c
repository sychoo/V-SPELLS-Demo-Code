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

void receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set);
void handle_heartbeat(const mavlink_message_t* message);
void handle_battery_status(const mavlink_message_t* message);
void handle_mission_count(const mavlink_message_t* message);

void send_some(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void initial_command(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void test_send_command(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);

void send_heartbeat(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void set_guided_mode(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void arm(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void takeoff(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void start_mission(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void start_geofence(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void define_geofence(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void start_geofence_mission_count(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void fence_list(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void define_fence_breach_action(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void handle_fence_breach(const mavlink_message_t* message);

int radius = 500;

int main(int argc, char* argv[])
{
    // establish 
    const int socket_fd = socket(PF_INET, SOCK_DGRAM, 0);

    if (socket_fd < 0) {
        printf("socket error: %s\n", strerror(errno));
        return -1;
    }

    struct sockaddr_in addr = {};
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, "0.0.0.0", &(addr.sin_addr));
    addr.sin_port = htons(14550); 

    if (bind(socket_fd, (struct sockaddr*)(&addr), sizeof(addr)) != 0) {
        printf("bind error: %s\n", strerror(errno));
        return -2;
    }
    printf("binded");

    struct sockaddr_in src_addr = {};
    socklen_t src_addr_len = sizeof(src_addr);
    bool src_addr_set = false;

    int init_cmd_req_count = 1;
    while (init_cmd_req_count > 0) {
        receive_some(socket_fd, &src_addr, &src_addr_len, &src_addr_set);
        initial_command(socket_fd, &src_addr, src_addr_len);
        init_cmd_req_count = init_cmd_req_count - 1;
    }

    while (true) {
        receive_some(socket_fd, &src_addr, &src_addr_len, &src_addr_set);

        if (src_addr_set) {
            send_some(socket_fd, &src_addr, src_addr_len);
        }
    }

    return 0;
}



void receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set)
{
    // printf("Receiving ...\n");
    // We just receive one UDP datagram and then return again.
    char buffer[2048]; // enough for MTU 1500 bytes

    const int ret = recvfrom(
            socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)(src_addr), src_addr_len);

    if (ret < 0) {
        printf("recvfrom error: %s\n", strerror(errno));
    } else if (ret == 0) {
        // peer has done an orderly shutdown
        return;
    } 

    *src_addr_set = true;

    mavlink_message_t message;
    mavlink_status_t status;
    for (int i = 0; i < ret; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status) == 1) {

            // printf(
            //     "Received message %d from %d/%d\n",
            //     message.msgid, message.sysid, message.compid);

            switch (message.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                handle_heartbeat(&message);
                break;
            case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
                printf("Received fence point\n");
                break;
            case MAVLINK_MSG_ID_BATTERY_STATUS:
                // printf("Received battery status\n");
                handle_battery_status(&message);
                break;
            case MAVLINK_MSG_ID_MISSION_COUNT:
                printf("Received MISSION COUNT\n");
                handle_mission_count(&message);
                break;
            case MAVLINK_MSG_ID_FENCE_STATUS:
                printf("Received FENCE BREACH\n");
                handle_fence_breach(&message);
                break;
            
            }

        }
    }
}
// https://mavlink.io/en/messages/common.html#FENCE_BREACH
void handle_fence_breach(const mavlink_message_t* message)
{
    mavlink_fence_status_t fence_status;
    // mavlink_msg_heartbeat_decode(message, &heartbeat);
    mavlink_msg_fence_status_decode(message, &fence_status);

    printf("Got Fence Status: ");
    printf("Fence Status: %d\n", fence_status.breach_type);
    switch(fence_status.breach_type) {
        case FENCE_BREACH_NONE:
            printf("No breach\n");
            break;
        case FENCE_BREACH_MINALT:
            printf("Minimum Altitude Breach\n");
            break;

        case FENCE_BREACH_MAXALT:
            printf("Maximum Altitude Breach\n");
            break;  

        case FENCE_BREACH_BOUNDARY:
            printf("Boundary Breach\n");
            break;
    }
}

// https://mavlink.io/en/services/battery.html
void handle_mission_count(const mavlink_message_t* message)
{
    mavlink_mission_count_t batteryinfo;
    // mavlink_msg_heartbeat_decode(message, &heartbeat);
    mavlink_msg_mission_count_decode(message, &batteryinfo);

    printf("Got Mission Count: ");
    printf("Mission Count: %d\n", batteryinfo.count);
    // switch (heartbeat.autopilot) {
    //     case MAV_AUTOPILOT_GENERIC:
    //         printf("generic");
    //         break;
    //     case MAV_AUTOPILOT_ARDUPILOTMEGA:
    //         printf("ArduPilot");
    //         break;
    //     case MAV_AUTOPILOT_PX4:
    //         printf("PX4");
    //         break;
    //     default:
    //         printf("other");
    //         break;
    // }
    // printf(" autopilot\n");
}

// https://mavlink.io/en/services/battery.html
void handle_battery_status(const mavlink_message_t* message)
{
    mavlink_battery_status_t batteryinfo;
    // mavlink_msg_heartbeat_decode(message, &heartbeat);
    mavlink_msg_battery_status_decode(message, &batteryinfo);

    printf("Got battery information: ");
    printf("Battery Remaining: %d\n", batteryinfo.battery_remaining);
    // switch (heartbeat.autopilot) {
    //     case MAV_AUTOPILOT_GENERIC:
    //         printf("generic");
    //         break;
    //     case MAV_AUTOPILOT_ARDUPILOTMEGA:
    //         printf("ArduPilot");
    //         break;
    //     case MAV_AUTOPILOT_PX4:
    //         printf("PX4");
    //         break;
    //     default:
    //         printf("other");
    //         break;
    // }
    // printf(" autopilot\n");
}

void handle_heartbeat(const mavlink_message_t* message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(message, &heartbeat);

    printf("Got heartbeat from ");
    switch (heartbeat.autopilot) {
        case MAV_AUTOPILOT_GENERIC:
            printf("generic");
            break;
        case MAV_AUTOPILOT_ARDUPILOTMEGA:
            printf("ArduPilot");
            break;
        case MAV_AUTOPILOT_PX4:
            printf("PX4");
            break;
        default:
            printf("other");
            break;
    }
    printf(" autopilot\n");
}

void send_some_init(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    // Whenever a second has passed, we send a heartbeat.
    static time_t last_time = 0;
    time_t current_time = time(NULL);
    if (current_time - last_time >= 1) {
        initial_command(socket_fd, src_addr, src_addr_len);

        last_time = current_time;
    }
}

void send_some(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    // Whenever a second has passed, we send a heartbeat.
    static time_t last_time = 0;
    time_t current_time = time(NULL);
    if (current_time - last_time >= 1) {
        // send_heartbeat(socket_fd, src_addr, src_addr_len);
        test_send_command(socket_fd, src_addr, src_addr_len);

        last_time = current_time;
    }
}



void send_heartbeat(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;

    const uint8_t system_id = 42;
    const uint8_t base_mode = 0;
    const uint8_t custom_mode = 0;


    // [works] sending heartbeat
    mavlink_msg_heartbeat_pack_chan(
        system_id,
        MAV_COMP_ID_PERIPHERAL,
        MAVLINK_COMM_0,
        &message,
        MAV_TYPE_GENERIC,
        MAV_AUTOPILOT_GENERIC,
        base_mode,
        custom_mode,
        MAV_STATE_STANDBY);


    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Sent heartbeat\n");
    }
}


// note that system = 1, target = 0. otherwise no response or ack from ardupilot
void set_guided_mode(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;
    mavlink_command_long_t set_mode = {0};
     
    set_mode.target_system = 1; // must be 1 
	set_mode.target_component = 0; // must be 0
	set_mode.command = MAV_CMD_DO_SET_MODE;		// 176
	set_mode.confirmation = true;
	set_mode.param1 = 1; 				//need to be 1 ?? check			 	
	set_mode.param2 = 4; // 4 is GUIDED mode for drones (https://ardupilot.org/copter/docs/parameters.html#fltmode1)
// 9 for landing mode
    mavlink_msg_command_long_encode(1, 0, &message, &set_mode);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Set GUIDED Mode\n");
    }
}


void arm(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;
     
    mavlink_command_long_t armed = {0};
	armed.target_system = 1;
	armed.target_component = 0;
	armed.command = MAV_CMD_COMPONENT_ARM_DISARM; //400
	armed.confirmation = true;
	armed.param1 = 1; // states (ready = 1)
	
	// Encode:
	mavlink_msg_command_long_encode(1, 255, &message, &armed);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Drone is Armed\n");
    }
}

// note that parameter 7 determines the takeoff altitude
void takeoff(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;

    mavlink_command_long_t takeoff = {0};
	takeoff.target_system = 1;
	takeoff.target_component = 0;
	takeoff.command = MAV_CMD_NAV_TAKEOFF; //400
	takeoff.confirmation = true;
	takeoff.param7 = 20; //takeoff altitude in meters
	
	// Encode:
	mavlink_msg_command_long_encode(1, 255, &message, &takeoff);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Drone is Taking Off\n");
    }
}


void start_geofence(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;
     
    mavlink_command_long_t armed = {0};
	armed.target_system = 1;
	armed.target_component = 0;
	armed.command = MAV_CMD_DO_FENCE_ENABLE; //400
	armed.confirmation = true;
	armed.param1 = 1; // states (ready = 1)
	
	// Encode:
	mavlink_msg_command_long_encode(1, 255, &message, &armed);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("GeoFence is Enabled for the Drone\n");
    }
}

// MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
// the way to properly start a fence is by uploading the fence points and enable it (it must first signal how many waypoints it should receive before actually sending them)
// https://discuss.ardupilot.org/t/mavlink-mission-command-error-mav-mission-error-1/19286/4
void start_geofence_mission_count(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
	mavlink_mission_count_t mission = {0};
	mission.target_system = 1;
	mission.target_component = 0;
    mission.mission_type = MAV_MISSION_TYPE_FENCE;
    mission.count = 1; // number of messages in the sequence (fence points)
	
	// Encode:
	mavlink_message_t msg;

    mavlink_msg_mission_count_encode(1, 255, &msg, &mission);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &msg);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("GeoFence is Defined for the Drone\n");
    }
}


// MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
void define_geofence(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    // https://discuss.ardupilot.org/t/mavlink-mission-command-error-mav-mission-error-1/19286/4
	mavlink_mission_item_int_t mission = {0};
	mission.target_system = 1;
	mission.target_component = 0;
	mission.command = MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION; //; 	//213 (MAV_CMD_DO_SET_POSITION_YAW_THRUST)
    mission.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT; //MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    mission.current = 2;
    mission.autocontinue = 0;
    mission.mission_type = MAV_MISSION_TYPE_FENCE;

	mission.param1 = radius; // 200 meters
    // radius = radius - 10;
    mission.param2 = 0; // 0 group included
 	mission.x = (int) (-35.36277334 * 10000000.0);	 			//speed normalized [0 - 1]
    mission.y = (int) (149.16536671 * 10000000.0);	 	

	// Encode:
	mavlink_message_t msg;

    mavlink_msg_mission_item_int_encode(1, 255, &msg, &mission);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &msg);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("GeoFence is Defined for the Drone\n");
    }
}

// void sleep(int seconds) {
//   usleep(seconds * 1000000);
// }



// waypoint mission
// note that mission can only be started using the mavlink_mission_item)int construct in ArduPilot
void start_mission(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len) {
	mavlink_mission_item_int_t mission = {0};
	mission.target_system = 1;
	mission.target_component = 0;
	mission.command = MAV_CMD_NAV_WAYPOINT; //; 	//213 (MAV_CMD_DO_SET_POSITION_YAW_THRUST)
    mission.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT; //MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    mission.current = 2,
    mission.autocontinue = 0,
	mission.param1 = 10;			 	//angle (centridegree) [-4500 - 4500]
	mission.param2 = 10;			 	//angle (centridegree) [-4500 - 4500]
	mission.param3 = 0;			 	//angle (centridegree) [-4500 - 4500]
	mission.param4 = 0;			 	//angle (centridegree) [-4500 - 4500]

	mission.x = (int) (-35.313553 * 10000000.0);	 			//speed normalized [0 - 1]
    mission.y = (int) (149.162057 * 10000000.0);	 			//speed normalized [0 - 1]
    mission.z = 20;	 			//speed normalized [0 - 1]
	
	// Encode:
	mavlink_message_t msg;

    mavlink_msg_mission_item_int_encode(1, 255, &msg, &mission);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &msg);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Mission Started! Waypoint = (-35.313553, 149.162057)\n");
    }
}

void fence_list(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_fence_fetch_point_t fence = {0};
    fence.target_system = 1;
	fence.target_component = 0;
	fence.idx = 0; 

	// Encode:
	mavlink_message_t msg;

    mavlink_msg_fence_fetch_point_encode(1, 255, &msg, &fence);

	// // armed.command = MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION; //400
    // armed.command=MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &msg);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("GeoFence is Defined for the Drone\n");
    }
}

void clear_fence(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_param_set_t param = {0};
    param.target_system = 1;
	param.target_component = 0;
    param.param_type = MAV_PARAM_TYPE_UINT16;
    param.param_value = 0;
    strcpy(param.param_id, "FENCE_TOTAL");

	// Encode:
	mavlink_message_t msg;

    mavlink_msg_param_set_encode(1, 255, &msg, &param);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &msg);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("GeoFence is Defined for the Drone\n");
    }
}

void define_fence_breach_action(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_param_set_t param = {0};
    param.target_system = 1;
	param.target_component = 0;
    param.param_type = MAV_PARAM_TYPE_UINT16;
    param.param_value = FENCE_ACTION_HOLD;
    strcpy(param.param_id, "FENCE_ACTION");

	// Encode:
	mavlink_message_t msg;

    mavlink_msg_param_set_encode(1, 255, &msg, &param);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &msg);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("GeoFence is Defined for the Drone\n");
    }
}

void request_fence_points(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;
     
    mavlink_mission_request_list_t armed = {0};
	armed.target_system = 1;
	armed.target_component = 0;
    armed.mission_type = MAV_MISSION_TYPE_FENCE;
	

	// Encode:
	// mavlink_msg_command_long_encode(1, 255, &message, &armed);
    mavlink_msg_mission_request_list_encode(1, 255, &message, &armed);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Start Requesting Fence Points\n");
    }
}

// how MAVLink download work
// https://mavlink.io/en/services/mission.html#download_mission
// https://mavlink.io/en/services/mission.html#download_mission
// https://mavlink.io/en/mavgen_python/howto_requestmessages.html
void request_fence_point_1(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    // mavlink_message_t message;
     
    // mavlink_command_long_t armed = {0};
	// armed.target_system = 1;
	// armed.target_component = 0;
	// armed.command = MAV_CMD_REQUEST_MESSAGE; //400
	// armed.confirmation = true;
	// // armed.param1 = MAVLINK_MSG_ID_FENCE_FETCH_POINT_LEN; // states (ready = 1)
    // armed.param1 = MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
	
    mavlink_message_t message;
     
    mavlink_mission_request_int_t armed = {0};
	armed.target_system = 1;
	armed.target_component = 0;
    armed.mission_type = MAV_MISSION_TYPE_FENCE;
    armed.seq = 0;

    mavlink_msg_mission_request_int_encode(1, 255, &message, &armed);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Requesting First Mission Item\n");
    }
}
void send_mission_ack(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;
     
    mavlink_mission_ack_t armed = {0};
	armed.target_system = 1;
	armed.target_component = 0;
    armed.mission_type = MAV_MISSION_ACCEPTED;
    armed.type = 0;

    mavlink_msg_mission_ack_encode(1, 255, &message, &armed);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Mission Acknowledgement Sent\n");
    }
}


// note that parameter 7 determines the takeoff altitude
void move_drone_local(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;

    mavlink_set_position_target_local_ned_t sp = {0};
    sp.time_boot_ms = 0;
    sp.target_system = 1;
    sp.target_component = 0;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    sp.type_mask = 0b110111000111;
    sp.vx       = 5.0;
    sp.vy       = 0.0;
    sp.vz       = 0.0;
        
	// Encode:
	mavlink_msg_set_position_target_local_ned_encode(1, 255, &message, &sp);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Drone is Flying East\n");
    }
}


void test_send_command(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len) {
    start_geofence_mission_count(socket_fd, src_addr, src_addr_len);
    define_geofence(socket_fd, src_addr, src_addr_len);
    start_geofence(socket_fd, src_addr, src_addr_len);
}

void initial_command(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    set_guided_mode(socket_fd, src_addr, src_addr_len);
    arm(socket_fd, src_addr, src_addr_len);
    takeoff(socket_fd, src_addr, src_addr_len);
    sleep(10);

    start_mission(socket_fd, src_addr, src_addr_len);
    define_fence_breach_action(socket_fd, src_addr, src_addr_len);

    start_geofence_mission_count(socket_fd, src_addr, src_addr_len);
    define_geofence(socket_fd, src_addr, src_addr_len);
}