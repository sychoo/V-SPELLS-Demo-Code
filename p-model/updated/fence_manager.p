// foreign function
// - distance_to_home

machine FenceManagement {
  var battery_machine: BatteryFailSafeModified;
  var geofence_machine: GeoFenceModified;

  var power_consumption_rate: float; // percentage per meter

  var battery_level: float;
  var drone_lat: float; 
  var drone_lon: float; 

  start state Init {
    entry {
      power_consumption_rate = 1.0; 
      drone_lat = -35.36277334;
      drone_lon = 149.16536671;

      goto MonitorBatteryLevel;
    }
  }

  state MonitorBatteryLevel {
    on eUpdateDroneBatteryRequest do (current_battery_level: float) {
      battery_level = current_battery_level;
    }

    on eFenceRadiusRequest do (request: tFenceRadiusRequest) {
      var current_distance_to_home: float;
      current_distance_to_home = distance_to_home(drone_lat, drone_lon);
      
      // modification: dynamically calculate the fence radius
      send request.source, eFenceRadiusResponse, (fenceRadius = ((battery_level / power_consumption_rate) + current_distance_to_home) / 2.0,);
    }
  }
}