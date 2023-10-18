// foreign function
// - define_fence
// - request_drone_telemetry
// - return_to_home
// - distance_to_home

event eStartGeoFence;
event eRequestDroneTelemetry;
event eVerifyFenceBreach;
event eReturnToHome;

machine GeoFence
{
  var fence_radius : int; 
  var drone_lat : float; 
  var drone_lon : float;

  start state Init {
    entry {
      fence_radius = 100;
      drone_lat = -35.36277334;
      drone_lon = 149.16536671;
      send this, eStartGeoFence;
      goto RequestDroneTelemetry;
    }
  }

  state DefineFence {
    // modification
    send fenceManager, eFenceRadiusRequest, (source = this,);

    // modfication
    on eFenceRadiusResponse do (fence_radius_response : float) {
      define_fence(fence_radius_response);
    }

    on eStartGeoFence do {
      define_fence(fence_radius);
      send this, eRequestDroneTelemetry;
      goto RequestDroneTelemetry;
    }
  }

  state RequestDroneTelemetry {
    on eRequestDroneTelemetry do {
      drone_lat = request_drone_telemetry_lat();
      drone_lon = request_drone_telemetry_lon();

      send this, eVerifyFenceBreach;
      goto VerifyFenceBreach;
    }
  }

  state VerifyFenceBreach {
    on eVerifyFenceBreach do {
      if (distance_to_home(drone_lat, drone_lon) > fence_radius) {
        send this, eReturnToHome;
        goto ReturnToHome;
      }
      else {
        send this, eRequestDroneTelemetry;
        goto DefineFence;
      }
    }
  }

  state ReturnToHome {
    on eReturnToHome do {
      return_to_home();
    }
  }
}   