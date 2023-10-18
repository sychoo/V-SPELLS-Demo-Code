// foreign function
// - retrieve_battery
// - emergency_land

event eStartBatteryFailSafe;
event eRequestBatteryLevel;
event eUpdateBatteryLevel: float;
event eEmergencyLanding;

machine BatteryFailSafe
{
  var battery_level : int;
  var landing_threshold : int;

  start state Init {
    entry (landing_threshold_local : int) {
      battery_level = 100;
      landing_threshold = landing_threshold_local;
      send this, eStartBatteryFailSafe, (landing_threshold_amount = landing_threshold_local,);
      goto RequestBatteryLevel;
    }
  }

  state RequestBatteryLevel {
    var current_battery_level: float;

    on eStartBatteryFailSafe, eRequestBatteryLevel do {
      current_battery_level = retrieve_battery();
      send this, eUpdateBatteryLevel, current_battery_level;
    }
  }

  state UpdateBatteryLevel {
    on eUpdateBatteryLevel do (current_battery_level: float){
      battery_level = current_battery_level;

      // modification
      send fenceManager, eUpdateBatteryLevel, battery_level;

      if (battery_level < landing_threshold) {
        send this, eEmergencyLanding;
        goto eEmergencyLanding;
      }
      else {
        send this, eRequestBatteryLevel;
        goto RequestBatteryLevel;
      }
  }

  state EmergencyLanding {
    var response: bool;
    on eEmergencyLanding do {
      emergency_land();
    }
  }
  }