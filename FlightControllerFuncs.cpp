#include "FlightControllerFuncs.h"
#include "FlightSystem.h"

// #include <tinyxml2.h>

#include <atomic>
#include <iostream>
#include <future>
#include <vector>
#include <map>
#include <memory>
#include <thread>
#include <chrono>
#include <cstdint>
// #include <time.h>

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
// auto _mavsdk = std::make_unique<mavsdk::Mavsdk>();
// std::shared_ptr<mavsdk::System> _system;
// std::shared_ptr<mavsdk::Telemetry> _telemetry;
// std::shared_ptr<mavsdk::Action> _action;
// std::shared_ptr<mavsdk::Mission> _mission;

// std::atomic_bool timer_flag = true;
// std::thread timer_thread;
// std::vector<mavsdk::Mission::MissionItem> _missionItems;

PRT_MACHINEINST* contextM = NULL;
PRT_VALUE* PTMP_tmp1 = NULL; 
PRT_VALUE* PTMP_tmp10 = NULL;
PRT_VALUE* PTMP_tmp2_20 = NULL;

void receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set);
void handle_heartbeat(const mavlink_message_t* message);
void handle_battery_status(const mavlink_message_t* message);
void handle_mission_count(const mavlink_message_t* message);

void send_some(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void initial_command(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);

// void sleep(int seconds);
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

int radius = 500;

void send_heartbeat(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
// std::shared_ptr<mavsdk::System> get_system(mavsdk::Mavsdk& mavsdk)
// {
//     std::cout << "<PrintLog> Waiting to discover system...\n";
//     auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
//     auto fut = prom.get_future();

//     mavsdk.subscribe_on_new_system([&mavsdk, &prom]() 
// 	{
//         auto system = mavsdk.systems().back();

//         if (system->has_autopilot()) 
// 		{
//             std::cout << "<PrintLog> Discovered autopilot\n";

//             mavsdk.subscribe_on_new_system(nullptr);
//             prom.set_value(system);
//         }
//     });

//     if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) 
// 	{
//         std::cerr << "<ErrorLog> No autopilot found.\n";
//         return {};
//     }

//     return fut.get();
// }

// // Init foreign function
// PRT_VALUE* P_CoreSetupMavSDK_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs)
// {	
// 	mavsdk::ConnectionResult connection_result = _mavsdk->add_any_connection("udp://:14540");

//     if (connection_result != mavsdk::ConnectionResult::Success) 
// 	{
//         std::cerr << "<ErrorLog> Connection failed: " << connection_result << '\n';
//         return PrtMkBoolValue(PRT_FALSE);
//     }
// 	_system = get_system(*_mavsdk);
//     if (!_system) 
// 	{
//         return PrtMkBoolValue(PRT_FALSE);
//     }
	
//     _action = std::make_shared<mavsdk::Action>(_system);

//     _mission = std::make_shared<mavsdk::Mission>(_system);

//     _telemetry = std::make_shared<mavsdk::Telemetry>(_system);

// 	return PrtMkBoolValue(PRT_TRUE);
// }

// Reply foreign functions
// PRT_VALUE* P_ArmSystem_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs)
// {
//     mavsdk::Action::Result res = _action->arm();
//     if(res == mavsdk::Action::Result::Success)
//     {
//         return PrtMkBoolValue(PRT_TRUE);
//     }
//     return PrtMkBoolValue(PRT_FALSE);
// }

// PRT_VALUE* P_SetTakeoffHeight_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs)
// {
//     const mavsdk::Action::Result stares = _action->set_takeoff_altitude(15.0f);

//     if(stares == mavsdk::Action::Result::Success)
//     {
//         return PrtMkBoolValue(PRT_TRUE);
//     }
//     return PrtMkBoolValue(PRT_FALSE);
// }

// PRT_VALUE* P_TakeoffSystem_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs)
// {
// 	const mavsdk::Action::Result tores = _action->takeoff();

//     if(tores == mavsdk::Action::Result::Success)
//     {
//         return PrtMkBoolValue(PRT_TRUE);
//     }
//     return PrtMkBoolValue(PRT_FALSE);
// }

// PRT_VALUE* P_TelemetryHealthAllOk_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs)
// {
//     auto prom = std::promise<bool>{};
//     auto fut = prom.get_future();
//     _telemetry->subscribe_health_all_ok([&prom](bool health)
//     {
//         if(health)
//         {
//             _telemetry->subscribe_health_all_ok(nullptr);
//             prom.set_value(true);
//         }
//     });

//     if (fut.wait_for(std::chrono::seconds(30)) == std::future_status::timeout) 
// 	{
//         return PrtMkBoolValue((PRT_BOOLEAN)false);
//     }
//     return PrtMkBoolValue((PRT_BOOLEAN)fut.get());
// }

// PRT_VALUE* P_UploadMission_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs)
// {
//     _missionItems.clear();

//     tinyxml2::XMLDocument doc;
//     const char* path = "/Users/rockita/Code/vspells/PFlight/Res/mission.xml";
//     doc.LoadFile(path);

//     tinyxml2::XMLElement* root = doc.RootElement();
//     tinyxml2::XMLElement* pMI = root->FirstChildElement("MissionItem");
//     while(pMI)
//     {
//         mavsdk::Mission::MissionItem mi{};
//         pMI->FirstChildElement("Latitude")->QueryDoubleText(&mi.latitude_deg);
//         pMI->FirstChildElement("Longitude")->QueryDoubleText(&mi.longitude_deg);
//         pMI->FirstChildElement("RelativeAltitude")->QueryFloatText(&mi.relative_altitude_m);
//         pMI->FirstChildElement("Speed")->QueryFloatText(&mi.speed_m_s);
//         pMI->FirstChildElement("IsFlyThrough")->QueryBoolText(&mi.is_fly_through);
//         pMI->FirstChildElement("GimbalPitch")->QueryFloatText(&mi.gimbal_pitch_deg);
//         pMI->FirstChildElement("GimbalYaw")->QueryFloatText(&mi.gimbal_yaw_deg);
//         mi.camera_action = mavsdk::Mission::MissionItem::CameraAction::None;
//         pMI->FirstChildElement("LoiterTime")->QueryFloatText(&mi.loiter_time_s);
//         pMI->FirstChildElement("CameraPhotoInterval")->QueryDoubleText(&mi.camera_photo_interval_s);
//         pMI->FirstChildElement("AcceptanceRadius")->QueryFloatText(&mi.acceptance_radius_m);
//         pMI->FirstChildElement("Yaw")->QueryFloatText(&mi.yaw_deg);
//         pMI->FirstChildElement("CameraPhotoDistance")->QueryFloatText(&mi.camera_photo_distance_m);
//         _missionItems.push_back(mi);
//         pMI = pMI->NextSiblingElement("MissionItem");
//     }

//     std::cout << _missionItems[0].latitude_deg << std::endl;
//     mavsdk::Mission::MissionPlan mission_plan{};
//     mission_plan.mission_items = _missionItems;
//     const mavsdk::Mission::Result upload_result = _mission->upload_mission(mission_plan);
//     if (upload_result != mavsdk::Mission::Result::Success) 
//     {
//         return PrtMkBoolValue((PRT_BOOLEAN)false);
//     }
//     std::cout << "Uploaded" << std::endl;

//     return PrtMkBoolValue((PRT_BOOLEAN)true);
// }

// PRT_VALUE* P_SystemStatus_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs)
// {
//     return PrtMkBoolValue((PRT_BOOLEAN)_system->is_connected());
// }


PRT_VALUE* P_BatteryRemaining_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs)
{
    // code for sleeping
    //  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // mavlink_message_t message;
    // mavlink_command_long_t set_mode = {0};
     
    // set_mode.target_system = 1; // must be 1 
	// set_mode.target_component = 0; // must be 0
	// set_mode.command = MAV_CMD_DO_SET_MODE;		// 176
	// set_mode.confirmation = true;
	// set_mode.param1 = 1; 				//need to be 1 ?? check			 	
	// set_mode.param2 = 4; // 4 is GUIDED mode for drones (https://ardupilot.org/copter/docs/parameters.html#fltmode1)

    // mavlink_msg_command_long_encode(1, 0, &message, &set_mode);

    // // write to the socket
    // uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    // const int len = mavlink_msg_to_send_buffer(buffer, &message);

    // int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    // if (ret != len) {
    //     printf("sendto error: %s\n", strerror(errno));
    // } else {
    //     printf("Set GUIDED Mode\n");
    // }
// }

    // auto prom = std::promise<float>{};
    // auto fut = prom.get_future();
    // _telemetry->subscribe_battery([&prom](mavsdk::Telemetry::Battery battery)
    // {
    //     _telemetry->subscribe_battery(nullptr);
    //     prom.set_value(battery.remaining_percent);
    // });

    // if (fut.wait_for(std::chrono::seconds(30)) == std::future_status::timeout) 
	// {
    //     return PrtMkFloatValue((PRT_FLOAT)0.0f);
    // }
    // return PrtMkFloatValue((PRT_FLOAT)fut.get());
    printf("Hello, world!\n");

     // Open UDP socket
    const int socket_fd = socket(PF_INET, SOCK_DGRAM, 0);

    if (socket_fd < 0) {
        printf("socket error: %s\n", strerror(errno));
        return PrtMkFloatValue((PRT_FLOAT)-1.0f);
    }

    // Bind to port
    struct sockaddr_in addr = {};
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, "0.0.0.0", &(addr.sin_addr)); // listen on all network interfaces
    addr.sin_port = htons(14550); // default port on the ground
    // memcpy(&addr.sin_port, hserver->h_addr, hserver->h_length);

    printf("binding...\n");

    if (bind(socket_fd, (struct sockaddr*)(&addr), sizeof(addr)) != 0) {
        printf("bind error: %s\n", strerror(errno));
        // return -2;
        PrtMkFloatValue((PRT_FLOAT)-2.0f);
    }
    printf("binded\n");
    // We set a timeout at 100ms to prevent being stuck in recvfrom for too
    // long and missing our chance to send some stuff.
    // struct timeval tv;
    // tv.tv_sec = 0;
    // tv.tv_usec = 100000;
    // if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    //     printf("setsockopt error: %s\n", strerror(errno));
    //     return -3;
    // }

    struct sockaddr_in src_addr = {};
    socklen_t src_addr_len = sizeof(src_addr);
    bool src_addr_set = false;

    // command that only needs to be executed once
    // initial_command(socket_fd, &src_addr, src_addr_len);

    while (true) {
        // For illustration purposes we don't bother with threads or async here
        // and just interleave receiving and sending.
        // This only works  if receive_some returns every now and then.
        receive_some(socket_fd, &src_addr, &src_addr_len, &src_addr_set);

        if (src_addr_set) {
            send_some(socket_fd, &src_addr, src_addr_len);
        }
    }

    return PrtMkFloatValue((PRT_FLOAT)5.0f);
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
            }
        }
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
    // fence action
    // https://mavlink.io/en/messages/common.html#FENCE_ACTION_RTL

    // mission type fence
    // https://mavlink.io/en/messages/common.html#MAV_MISSION_TYPE_FENCE

    // command to start the geo fence
    // https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-do-fence-enable

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
    // geofence inclusion
    // https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
    
    // fence action
    // https://mavlink.io/en/messages/common.html#FENCE_ACTION_RTL

    // mission type fence
    // https://mavlink.io/en/messages/common.html#MAV_MISSION_TYPE_FENCE

    // command to start the geo fence
    // https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-do-fence-enable

    // mavlink_message_t message;
     
     // go to waypoint: -35.313553, 149.162057

    // https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-nav-waypoint
    // https://discuss.ardupilot.org/t/got-command-ack-nav-waypoint-unsupported/93054/2
    // https://github.com/mustafa-gokce/ardupilot-software-development/blob/main/pymavlink/goto-location.py

    // https://discuss.ardupilot.org/t/mavlink-mission-command-error-mav-mission-error-1/19286/4
	mavlink_mission_count_t mission = {0};
	mission.target_system = 1;
	mission.target_component = 0;
    mission.mission_type = MAV_MISSION_TYPE_FENCE;
    mission.count = 1; // number of messages in the sequence (fence points)
	
	// Encode:
	mavlink_message_t msg;

    mavlink_msg_mission_count_encode(1, 255, &msg, &mission);

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


// MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
void define_geofence(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    // geofence inclusion
    // https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
    
    // fence action
    // https://mavlink.io/en/messages/common.html#FENCE_ACTION_RTL

    // mission type fence
    // https://mavlink.io/en/messages/common.html#MAV_MISSION_TYPE_FENCE

    // command to start the geo fence
    // https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-do-fence-enable

    // mavlink_message_t message;
     
     // go to waypoint: -35.313553, 149.162057

    // https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-nav-waypoint
    // https://discuss.ardupilot.org/t/got-command-ack-nav-waypoint-unsupported/93054/2
    // https://github.com/mustafa-gokce/ardupilot-software-development/blob/main/pymavlink/goto-location.py

    // https://discuss.ardupilot.org/t/mavlink-mission-command-error-mav-mission-error-1/19286/4
	mavlink_mission_item_int_t mission = {0};
	mission.target_system = 1;
	mission.target_component = 0;
	mission.command = MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION; //; 	//213 (MAV_CMD_DO_SET_POSITION_YAW_THRUST)
    mission.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT; //MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    mission.current = 2;
    mission.autocontinue = 0;
    mission.mission_type = MAV_MISSION_TYPE_FENCE;
	// mission.param1 = 10;			 	//angle (centridegree) [-4500 - 4500]
	// mission.param2 = 10;			 	//angle (centridegree) [-4500 - 4500]
	// mission.param3 = 0;			 	//angle (centridegree) [-4500 - 4500]
	// mission.param4 = 0;			 	//angle (centridegree) [-4500 - 4500]

	mission.param1 = radius; // 200 meters
    radius = radius - 10;
    mission.param2 = 0; // 0 group included
 	mission.x = (int) (-35.36277334 * 10000000.0);	 			//speed normalized [0 - 1]
    mission.y = (int) (149.16536671 * 10000000.0);	 	

	// mission.x = (int) (-35.313553 * 10000000.0);	 			//speed normalized [0 - 1]
    // mission.y = (int) (149.162057 * 10000000.0);	 			//speed normalized [0 - 1]
    // mission.z = 20;	 			//speed normalized [0 - 1]
	
	// Encode:
	mavlink_message_t msg;

    mavlink_msg_mission_item_int_encode(1, 255, &msg, &mission);

    //      mavlink_command_long_t armed = {0};
	// armed.target_system = 1;
	// armed.target_component = 0;
	// // armed.command = MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION; //400
    // armed.command=MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
	// // armed.confirmation = true;
	// armed.param1 = 200; // 200 meters
    // armed.param2 = 0; // 0 group included
 	// armed.param5 = (float) (-35.36277334 * 10000000.0);	 			//speed normalized [0 - 1]
    // armed.param6 = (float) (149.16536671 * 10000000.0);	 			//speed normalized [0 - 1]
    // // mission.z = 20;	 			//speed normalized [0 - 1]
	
	
	// // Encode:
	// mavlink_msg_command_long_encode(1, 255, &message, &armed);

    // mavlink_command_int_t armed = {0};
	// armed.target_system = 1;
	// armed.target_component = 0;
	// // armed.command = MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION; //400
    // armed.command=MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
	// // armed.confirmation = true;
	// armed.param1 = 200; // 200 meters
    // armed.param2 = 0; // 0 group included
 	// armed.x = (int) (-35.36277334 * 10000000.0);	 			//speed normalized [0 - 1]
    // armed.y = (int) (149.16536671 * 10000000.0);	 			//speed normalized [0 - 1]
    // // mission.z = 20;	 			//speed normalized [0 - 1]
	
	
	// // Encode:
	// mavlink_msg_command_int_encode(1, 255, &message, &armed);

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

    // go to waypoint: -35.313553, 149.162057

    // https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-nav-waypoint
    // https://discuss.ardupilot.org/t/got-command-ack-nav-waypoint-unsupported/93054/2
    // https://github.com/mustafa-gokce/ardupilot-software-development/blob/main/pymavlink/goto-location.py
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

// void sleep(int seconds) {
//   usleep(seconds * 1000000);
// }

// https://mavlink.io/en/messages/ardupilotmega#FENCE_FETCH_POINT
// fetch a fence point to display on the map
// https://github.com/ArduPilot/MAVProxy/blob/master/MAVProxy/modules/mavproxy_fenceitem_protocol.py#L256
// https://github.com/arktools/ardupilotone/blob/eea16ef1d6a0be2151886d6e5c3bed280cc4aea4/Tools/autotest/pymavlink/mavlink.py#L2383
void fence_list(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    // geofence inclusion
    // https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
    
    // fence action
    // https://mavlink.io/en/messages/common.html#FENCE_ACTION_RTL

    // mission type fence
    // https://mavlink.io/en/messages/common.html#MAV_MISSION_TYPE_FENCE

    // command to start the geo fence
    // https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-do-fence-enable

    // mavlink_message_t message;
     
     // go to waypoint: -35.313553, 149.162057

    // https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-nav-waypoint
    // https://discuss.ardupilot.org/t/got-command-ack-nav-waypoint-unsupported/93054/2
    // https://github.com/mustafa-gokce/ardupilot-software-development/blob/main/pymavlink/goto-location.py

    // https://discuss.ardupilot.org/t/mavlink-mission-command-error-mav-mission-error-1/19286/4
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


// how to set system parameter
// set_fence_total_param
// clear fence means setting the fence total parameters to 0
// https://github.com/ArduPilot/MAVProxy/blob/master/MAVProxy/modules/mavproxy_fenceitem_protocol.py#L256
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

// how MAVLink download work
// https://mavlink.io/en/services/mission.html#download_mission
// https://mavlink.io/en/mavgen_python/howto_requestmessages.html
void request_fence_points(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
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
     
    mavlink_mission_request_list_t armed = {0};
	armed.target_system = 1;
	armed.target_component = 0;
	// armed.command = MAV_CMD_REQUEST_MESSAGE; //400
    armed.mission_type = MAV_MISSION_TYPE_FENCE;
	// armed.confirmation = true;
	// armed.param1 = MAVLINK_MSG_ID_FENCE_FETCH_POINT_LEN; // states (ready = 1)
    // armed.param1 = MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
	

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
	// armed.command = MAV_CMD_REQUEST_MESSAGE; //400
    armed.mission_type = MAV_MISSION_TYPE_FENCE;
    armed.seq = 0;
	// armed.confirmation = true;
	// armed.param1 = MAVLINK_MSG_ID_FENCE_FETCH_POINT_LEN; // states (ready = 1)
    // armed.param1 = MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
	

	// Encode:
	// mavlink_msg_command_long_encode(1, 255, &message, &armed);
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
    // mavlink_message_t message;
     
    // mavlink_command_long_t armed = {0};
	// armed.target_system = 1;
	// armed.target_component = 0;
	// armed.command = MAV_CMD_REQUEST_MESSAGE; //400
	// armed.confirmation = true;
	// // armed.param1 = MAVLINK_MSG_ID_FENCE_FETCH_POINT_LEN; // states (ready = 1)
    // armed.param1 = MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
	
    mavlink_message_t message;
     
    mavlink_mission_ack_t armed = {0};
	armed.target_system = 1;
	armed.target_component = 0;
	// armed.command = MAV_CMD_REQUEST_MESSAGE; //400
    armed.mission_type = MAV_MISSION_ACCEPTED;
    armed.type = 0;
	// armed.confirmation = true;
	// armed.param1 = MAVLINK_MSG_ID_FENCE_FETCH_POINT_LEN; // states (ready = 1)
    // armed.param1 = MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
	

	// Encode:
	// mavlink_msg_command_long_encode(1, 255, &message, &armed);
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

// executed every second
void test_send_command(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len) {
    // set_guided_mode(socket_fd, src_addr, src_addr_len);
    // arm(socket_fd, src_addr, src_addr_len);
    // takeoff(socket_fd, src_addr, src_addr_len);
    // sleep(10);
    // start_mission(socket_fd, src_addr, src_addr_len);

    start_geofence_mission_count(socket_fd, src_addr, src_addr_len);
    define_geofence(socket_fd, src_addr, src_addr_len);

    // sleep(1);
    // // start_geofence(socket_fd, src_addr, src_addr_len);

    // fence_list(socket_fd, src_addr, src_addr_len);
    // clear_fence(socket_fd, src_addr, src_addr_len);
    // sleep(2);

    start_geofence_mission_count(socket_fd, src_addr, src_addr_len);
    define_geofence(socket_fd, src_addr, src_addr_len);

    // sleep(2);
    // sleep(2);
    // request_fence_points(socket_fd, src_addr, src_addr_len);
    // sleep(1);
    // usleep(100000);
    // usleep(500000);
    // sleep(1);
    // sleep(0.1);
    // request_fence_point_1(socket_fd, src_addr, src_addr_len);
    // usleep(500000);
    // sleep(1);
    // send_mission_ack(socket_fd, src_addr, src_addr_len);
    // sleep(1);
    // acknowledgement 
}


void initial_command(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    // set_guided_mode(socket_fd, src_addr, src_addr_len);
    // arm(socket_fd, src_addr, src_addr_len);
    // takeoff(socket_fd, src_addr, src_addr_len);
    // sleep(10);

    // // Whenever a second has passed, we send a heartbeat.
    // static time_t last_time = 0;
    // time_t current_time = time(NULL);
    // if (current_time - last_time >= 1) {
    //     // send_heartbeat(socket_fd, src_addr, src_addr_len);
    //     test_send_command(socket_fd, src_addr, src_addr_len);

    //     last_time = current_time;
    // }

    // transmit the fence definition
    // sleep(1);

}
