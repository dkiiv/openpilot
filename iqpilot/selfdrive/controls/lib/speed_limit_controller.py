#!/usr/bin/env python3
"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
import calendar
import json
import numpy as np

from concurrent.futures import ThreadPoolExecutor

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

from openpilot.iqpilot.common.slc_utilities import calculate_bearing_offset, is_url_pingable
from openpilot.iqpilot.common.slc_variables import (
  FREE_MAPBOX_REQUESTS,
  OFFSET_MAP_IMPERIAL,
  OFFSET_MAP_METRIC
)

try:
  import requests
except ImportError:
  requests = None


class SpeedLimitController:
  def __init__(self, params):
    self.params = params

    self.calling_mapbox = False
    self.override_slc = False

    self.denied_target = 0
    self.map_speed_limit = 0
    self.mapbox_limit = 0
    self.next_speed_limit = 0
    self.overridden_speed = 0
    self.segment_distance = 0
    self.speed_limit_changed_timer = 0
    self.target = 0
    self.unconfirmed_speed_limit = 0

    self.previous_source = "None"
    self.source = "None"
    self.active_source = "None"
    self.active_target = 0.0

    self.speed_limit_accepted = False

    self.gps_valid = False
    self.gps_position = {
      "bearing": 0,
      "latitude": 0,
      "longitude": 0
    }

    mapbox_requests_raw = self.params.get("MapBoxRequests")
    if mapbox_requests_raw is not None:
      try:
        if isinstance(mapbox_requests_raw, bytes):
          self.mapbox_requests = json.loads(mapbox_requests_raw.decode('utf-8'))
        else:
          self.mapbox_requests = json.loads(mapbox_requests_raw)
      except (json.JSONDecodeError, AttributeError):
        self.mapbox_requests = {}
    else:
      self.mapbox_requests = {}
    self.mapbox_requests.setdefault("total_requests", 0)
    self.mapbox_requests.setdefault("max_requests", FREE_MAPBOX_REQUESTS - (28 * 100))

    self.mapbox_host = "https://api.mapbox.com"
    self.mapbox_token = self.params.get("MapboxToken")
    if self.mapbox_token is not None and isinstance(self.mapbox_token, bytes):
      self.mapbox_token = self.mapbox_token.decode('utf-8')

    previous_limit = self.params.get("PreviousSpeedLimit")
    if previous_limit is not None:
      try:
        if isinstance(previous_limit, bytes):
          self.previous_target = float(previous_limit.decode('utf-8'))
        else:
          self.previous_target = float(previous_limit)
      except (ValueError, AttributeError):
        self.previous_target = 0.0
    else:
      self.previous_target = 0.0

    self.executor = ThreadPoolExecutor(max_workers=1)

    self.session = requests.Session() if requests is not None else None
    if self.session is not None:
      self.session.headers.update({"Accept-Language": "en"})
      self.session.headers.update({"User-Agent": "iqpilot-mapbox-speed-limit-retriever/1.0"})

  @staticmethod
  def _is_alive(sm, key: str) -> bool:
    if hasattr(sm, "alive"):
      return bool(sm.alive.get(key, False))
    return isinstance(sm, dict) and key in sm

  def get_offset(self, is_metric):
    offset_map = OFFSET_MAP_METRIC if is_metric else OFFSET_MAP_IMPERIAL
    for low, high, offset_param in offset_map:
      if low < self.target < high:
        offset_value = self.params.get(offset_param)
        if offset_value is not None:
          if isinstance(offset_value, bytes):
            return float(offset_value.decode('utf-8'))
          return float(offset_value)
        return 0
    return 0

  def update_gps(self, sm):
    if self._is_alive(sm, "gpsLocationExternal"):
      gps_location = sm["gpsLocationExternal"]
    elif self._is_alive(sm, "gpsLocation"):
      gps_location = sm["gpsLocation"]
    else:
      gps_location = None

    if gps_location and hasattr(gps_location, 'flags') and gps_location.flags > 0:
      self.gps_valid = True
      self.gps_position = {
        "bearing": getattr(gps_location, 'bearingDeg', 0),
        "latitude": getattr(gps_location, 'latitude', 0),
        "longitude": getattr(gps_location, 'longitude', 0)
      }
    else:
      self.gps_valid = False

  def get_mapbox_speed_limit(self, now, time_validated, v_ego, sm):
    if requests is None or self.session is None:
      self.mapbox_limit = 0
      self.segment_distance = 0
      return

    if not self.gps_valid or not self.mapbox_token or (sm["carState"].steeringAngleDeg - sm["liveParameters"].angleOffsetDeg) >= 45:
      self.mapbox_limit = 0
      self.segment_distance = 0
      return

    if v_ego < 1:
      return

    if self.segment_distance > 0:
      self.segment_distance -= v_ego * DT_MDL
      return

    if self.calling_mapbox:
      self.segment_distance = v_ego
      return

    def make_request():
      try:
        self.calling_mapbox = True

        successful = False

        if not is_url_pingable(self.mapbox_host):
          self.segment_distance = 1000
          return None

        if time_validated:
          current_month = now.month
          if current_month != self.mapbox_requests.get("month"):
            self.mapbox_requests.update({
              "month": current_month,
              "total_requests": 0,
              "max_requests": FREE_MAPBOX_REQUESTS - calendar.monthrange(now.year, current_month)[1] * 100,
            })

        self.mapbox_requests["total_requests"] += 1
        self.params.put_nonblocking("MapBoxRequests", json.dumps(self.mapbox_requests))

        current_bearing = self.gps_position.get("bearing")
        current_latitude = self.gps_position.get("latitude")
        current_longitude = self.gps_position.get("longitude")

        future_latitude, future_longitude = calculate_bearing_offset(current_latitude, current_longitude, current_bearing, v_ego)

        url = (
          f"{self.mapbox_host}/matching/v5/mapbox/driving/"
          f"{current_longitude},{current_latitude};"
          f"{future_longitude},{future_latitude}.json"
        )

        mapbox_params = {
          "access_token": self.mapbox_token,
          "annotations": "maxspeed,distance",
          "geometries": "polyline6",
          "overview": "full",
          "steps": "false",
          "radiuses": "10;10",
          "tidy": "true",
        }

        response = self.session.get(url, params=mapbox_params, timeout=10)
        response.raise_for_status()

        successful = True

        return response.json()
      except Exception as exception:
        print(f"Unexpected error in Mapbox request: {exception}")
      finally:
        self.calling_mapbox = False

        if not successful:
          self.mapbox_limit = 0
          self.segment_distance = v_ego

          return None

    def complete_request(future):
      try:
        data = future.result()
        if data:
          matchings = data.get("matchings") or []
          if not matchings:
            self.mapbox_limit = 0
            self.segment_distance = v_ego
            return

          legs = (matchings[0] or {}).get("legs") or []
          if not legs:
            self.mapbox_limit = 0
            self.segment_distance = v_ego
            return

          annotation = legs[0].get("annotation") or {}

          distances = annotation.get("distance") or [v_ego]
          segment_distance = distances[0]

          speed_data = annotation.get("maxspeed", [])
          speed_limit_kph = 0
          if speed_data:
            first_segment_speed = speed_data[0]
            speed_limit_kph = (first_segment_speed.get("speed") if first_segment_speed.get("speed") != "none" else 0) or 0

          if speed_limit_kph > 0:
            self.mapbox_limit = speed_limit_kph * CV.KPH_TO_MS
            self.segment_distance = segment_distance
            return

        self.mapbox_limit = 0
        self.segment_distance = v_ego

      except Exception as exception:
        print(f"Mapbox Callback Error: {exception}")
        self.mapbox_limit = 0
        self.segment_distance = v_ego

    future = self.executor.submit(make_request)
    future.add_done_callback(complete_request)

  def handle_limit_change(self, desired_source, desired_target, sm, slc_params):
    self.speed_limit_changed_timer += DT_MDL

    car_state_iq = sm["iqCarState"]
    accel_pressed = getattr(car_state_iq, "accelPressed", False)
    speed_limit_accepted = (accel_pressed and sm["carControl"].longActive) or self.speed_limit_accepted

    decel_pressed = getattr(car_state_iq, "decelPressed", False)
    speed_limit_denied = decel_pressed or (self.speed_limit_changed_timer >= 30)

    if speed_limit_accepted:
      self.overridden_speed = 0

      self.source = desired_source
      self.target = desired_target

      self.speed_limit_accepted = False

    elif speed_limit_denied:
      self.denied_target = desired_target

      self.previous_source = desired_source
      self.previous_target = desired_target

    elif desired_target < self.target and not slc_params.get("speed_limit_confirmation_lower", False):
      self.source = desired_source
      self.target = desired_target

    elif desired_target > self.target and not slc_params.get("speed_limit_confirmation_higher", False):
      self.source = desired_source
      self.target = desired_target

    else:
      self.source = "None"
      self.unconfirmed_speed_limit = desired_target

    if self.target != self.previous_target and self.target > 0 and not speed_limit_denied:
      self.denied_target = 0

      self.previous_source = self.source
      self.previous_target = self.target

      self.params.put_nonblocking("PreviousSpeedLimit", float(self.target))

  def update_limits(self, dashboard_speed_limit, now, time_validated, v_cruise, v_ego, sm, slc_params):
    self.update_gps(sm)
    self.update_map_speed_limit(v_ego, sm, slc_params)

    if self.mapbox_requests["total_requests"] < self.mapbox_requests["max_requests"]:
      self.get_mapbox_speed_limit(now, time_validated, v_ego, sm)
    else:
      self.mapbox_limit = 0
      self.segment_distance = 0

    limits = {
      "Dashboard": dashboard_speed_limit,
      "Mapbox": self.mapbox_limit,
      "Map Data": self.map_speed_limit
    }
    filtered_limits = {source: limit for source, limit in limits.items() if limit >= 1}

    priority_highest = slc_params.get("speed_limit_priority_highest", False)
    priority_lowest = slc_params.get("speed_limit_priority_lowest", False)
    priority1 = slc_params.get("speed_limit_priority1", "Dashboard")
    priority2 = slc_params.get("speed_limit_priority2", "Mapbox")
    priority3 = slc_params.get("speed_limit_priority3", "Map Data")

    if priority_highest:
      desired_source = max(filtered_limits, key=filtered_limits.get, default="None")
      desired_target = filtered_limits.get(desired_source, 0)

    elif priority_lowest:
      desired_source = min(filtered_limits, key=filtered_limits.get, default="None")
      desired_target = filtered_limits.get(desired_source, 0)

    elif filtered_limits:
      for priority in [priority1, priority2, priority3]:
        if priority in filtered_limits:
          desired_source = priority
          desired_target = filtered_limits[desired_source]
          break
      else:
        desired_source = "None"
        desired_target = 0

    else:
      desired_source = "None"
      desired_target = 0

    if desired_target == 0 or self.target == 0:
      if self.denied_target != self.previous_target > 0 and slc_params.get("slc_fallback_previous_speed_limit", False):
        desired_source = self.previous_source
        desired_target = self.previous_target

        self.target = desired_target

      elif sm["selfdriveState"].enabled and slc_params.get("slc_fallback_set_speed", False):
        desired_source = "None"
        desired_target = v_cruise

    self.active_source = desired_source
    self.active_target = float(desired_target)

    if abs(desired_target - self.previous_target) >= 1:
      self.handle_limit_change(desired_source, desired_target, sm, slc_params)
    elif desired_source != self.source and abs(desired_target - self.target) < 1:
      self.source = desired_source
    else:
      self.speed_limit_changed_timer = 0
      self.unconfirmed_speed_limit = 0

  def update_map_speed_limit(self, v_ego, sm, slc_params):

    if self._is_alive(sm, "iqLiveMapData"):
      map_data = sm["iqLiveMapData"]
      self.map_speed_limit = getattr(map_data, "speedLimit", 0) if getattr(map_data, "speedLimitValid", False) else 0
      self.next_speed_limit = getattr(map_data, "speedLimitAhead", 0) if getattr(map_data, "speedLimitAheadValid", False) else 0

      if self.next_speed_limit > 0:
        if self.map_speed_limit < self.next_speed_limit:
          lookahead_higher = slc_params.get("map_speed_lookahead_higher", 5)
          max_lookahead = lookahead_higher * v_ego
        elif self.map_speed_limit > self.next_speed_limit:
          lookahead_lower = slc_params.get("map_speed_lookahead_lower", 5)
          max_lookahead = lookahead_lower * v_ego
        else:
          max_lookahead = 0

        next_distance = getattr(map_data, "speedLimitAheadDistance", 0)
        if next_distance < max_lookahead:
          self.map_speed_limit = self.next_speed_limit
    else:
      self.map_speed_limit = 0
      self.next_speed_limit = 0

  def update_override(self, v_cruise, v_cruise_diff, v_ego, v_ego_diff, sm, slc_params, is_metric):
    offset = self.get_offset(is_metric)

    self.override_slc = self.overridden_speed > self.target + offset > 0
    self.override_slc |= sm["carState"].gasPressed and v_ego > self.target + offset > 0
    self.override_slc &= sm["selfdriveState"].enabled

    if self.override_slc:
      if slc_params.get("speed_limit_controller_override_manual", False):
        if sm["carState"].gasPressed:
          self.overridden_speed = max(v_ego + v_ego_diff, self.overridden_speed)
        self.overridden_speed = float(np.clip(self.overridden_speed, self.target + offset, v_cruise + v_cruise_diff))
      elif slc_params.get("speed_limit_controller_override_set_speed", False):
        self.overridden_speed = v_cruise + v_cruise_diff

      self.source = "None"
    else:
      self.overridden_speed = 0
