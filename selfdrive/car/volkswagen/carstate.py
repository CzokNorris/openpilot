import numpy as np
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.volkswagen.values import DBC_FILES, CANBUS, NetworkLocation, TransmissionType, GearShifter, BUTTON_STATES, CarControllerParams

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC_FILES.mqb)
    
    #self.ACC = PQacc()

    ### START OF MAIN CONFIG OPTIONS ###
    ### Do NOT modify here, modify in /data/bb_openpilot.cfg and reboot
    self.useTeslaRadar = CP.enableGasInterceptor
    self.radarVIN = "5YJSB7E17HF207544" # carlos_ddd
    self.radarOffset = 0.
    self.radarPosition = 1
    self.radarEpasType = 3
    ### END OF MAIN CONFIG OPTIONS ###

    # Configure for PQ35/PQ46/NMS network messaging
    self.get_can_parser = self.get_pq_can_parser
    self.get_cam_can_parser = self.get_pq_cam_can_parser
    self.update = self.update_pq
    self.gsaHystActive = False   # gearshift assistant hysteris
    self.gsaIntvActive = False
    self.gsaSpeedFreeze = 0.0
    if CP.transmissionType == TransmissionType.automatic:
      self.shifter_values = can_define.dv["Getriebe_1"]['Waehlhebelposition__Getriebe_1_']
    if CP.enableGasInterceptor:
      self.openpilot_enabled = False
    

    self.buttonStates = BUTTON_STATES.copy()

  def update_pq(self, pt_cp, cam_cp, acc_cp, trans_type):
    ret = car.CarState.new_message()
    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds.fl = pt_cp.vl["Bremse_3"]['Radgeschw__VL_4_1'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = pt_cp.vl["Bremse_3"]['Radgeschw__VR_4_1'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = pt_cp.vl["Bremse_3"]['Radgeschw__HL_4_1'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = pt_cp.vl["Bremse_3"]['Radgeschw__HR_4_1'] * CV.KPH_TO_MS

    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)


    ret.standstill = ret.vEgoRaw < 0.1

    # Update steering angle, rate, yaw rate, and driver input torque. VW send
    # the sign/direction in a separate signal so they must be recombined.
    ret.steeringAngle = pt_cp.vl["Lenkhilfe_3"]['LH3_BLW'] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]['LH3_BLWSign'])]
    ret.steeringRate = pt_cp.vl["Lenkwinkel_1"]['Lenkradwinkel_Geschwindigkeit'] * (1, -1)[int(pt_cp.vl["Lenkwinkel_1"]['Lenkradwinkel_Geschwindigkeit_S'])]
    ret.steeringTorque = pt_cp.vl["Lenkhilfe_3"]['LH3_LM'] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]['LH3_LMSign'])]
    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["Bremse_5"]['Giergeschwindigkeit'] * (1, -1)[int(pt_cp.vl["Bremse_5"]['Vorzeichen_der_Giergeschwindigk'])] * CV.DEG_TO_RAD

    # Update gas, brakes, and gearshift.
    if not self.CP.enableGasInterceptor:
      ret.gas = pt_cp.vl["Motor_3"]['Fahrpedal_Rohsignal'] / 100.0
      ret.gasPressed = ret.gas > 0
    else:
      ret.gas = (cam_cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS'] + cam_cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS2']) / 2.
      ret.gasPressed = ret.gas > 468

    ret.brake = pt_cp.vl["Bremse_5"]['Bremsdruck'] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["Motor_2"]['Bremstestschalter'])
    ret.brakeLights = bool(pt_cp.vl["Motor_2"]['Bremslichtschalter'])

    # Additional safety checks performed in CarInterface.
    self.parkingBrakeSet = bool(pt_cp.vl["Kombi_1"]['Bremsinfo'])  # FIXME: need to include an EPB check as well
    ret.espDisabled = bool(pt_cp.vl["Bremse_1"]['ESP_Passiv_getastet'])
    ret.espIntervention = bool(pt_cp.vl["Bremse_1"]['ESP_Eingriff']) or bool(pt_cp.vl["Bremse_1"]['ASR_Anforderung'])


    # Update gear and/or clutch position data.
    if trans_type == TransmissionType.automatic:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["Getriebe_1"]['Waehlhebelposition__Getriebe_1_'], None))
    elif trans_type == TransmissionType.manual:
      ret.clutchPressed = not pt_cp.vl["Motor_1"]['Kupplungsschalter']
      reverse_light = bool(pt_cp.vl["Gate_Komf_1"]['GK1_Rueckfahr'])
      if reverse_light:
        ret.gearShifter = GEAR.reverse
      else:
        ret.gearShifter = GEAR.drive
      self.engineRPM = pt_cp.vl["Motor_1"]['Motordrehzahl']  # engine RPM for gear shift assist
#      self.gearDesired = pt_cp.vl["Getriebe_2"]['eingelegte_Fahrstufe']                 # gear ECU wants                  # 2do: needs to be added to signals / checks
#      self.gearCurrent = pt_cp.vl["Getriebe_2"]['Ganganzeige_Kombi___Getriebe_Va']      # gear ECU detected

    # Update door and trunk/hatch lid open status.
    # TODO: need to locate signals for other three doors if possible
    ret.doorOpen = bool(pt_cp.vl["Gate_Komf_1"]['GK1_Fa_Tuerkont'])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = not bool(pt_cp.vl["Airbag_1"]["Gurtschalter_Fahrer"])

    # Update driver preference for metric. VW stores many different unit
    # preferences, including separate units for for distance vs. speed.
    # We use the speed preference for OP.
    self.displayMetricUnits = not pt_cp.vl["Einheiten_1"]["MFA_v_Einheit_02"]

    self.ldw_lane_warning_left = False
    self.ldw_lane_warning_right = False
    self.ldw_side_dlc_tlc = False
    self.ldw_dlc = False
    self.ldw_tlc = False
    
    # Update control button states for turn signals and ACC controls.
    self.buttonStates["accelCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Up_kurz'])
    self.buttonStates["decelCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Down_kurz'])
    self.buttonStates["cancel"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Abbrechen'])
    self.buttonStates["setCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Neu_Setzen'])
    self.buttonStates["resumeCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Recall'])
    self.buttonStates["gapAdjustCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Zeitluecke'])
    self.buttonStates["longUp"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Up_lang'])
    self.buttonStates["longDown"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Down_lang'])

    # Update ACC radar status.
    ret.cruiseState.available = bool(pt_cp.vl["GRA_Neu"]['GRA_Hauptschalt'])
    ret.graActive = True if pt_cp.vl["Motor_2"]['GRA_Status'] in [1, 2] else False
    
    # ACC emulation
    self.ACC_engaged, self.v_ACC = self.ACC.update_acc_iter_4CS(ret.vEgo*CV.MS_TO_KPH, self.buttonStates, ret.cruiseState.available, self.openpilot_enabled)

    # Engage open pilot if ACC emulation says so
    if self.CP.enableGasInterceptor and self.ACC_engaged:
      self.openpilot_enabled = True
    # if ACC emulation is not active we want OP still engaged but setspeed <= current speed
    elif self.CP.enableGasInterceptor and not self.ACC_engaged:
      self.v_ACC = (ret.vEgo - 1) * CV.MS_TO_KPH
    else:
      self.openpilot_enabled = False

    # Check if Gas or Brake pressed and override ACC emulation
    if self.CP.enableGasInterceptor and (ret.gasPressed or ret.brakePressed or ret.espIntervention):
      self.openpilot_enabled = False

    # Override openpilot enabled if gas interceptor installed
    if self.CP.enableGasInterceptor and self.openpilot_enabled:
      ret.cruiseState.enabled = True
    else:
      ret.cruiseState.enabled = False

    ret.cruiseState.speed = self.v_ACC * CV.KPH_TO_MS

    # for manual cars only (gearshift assistant)
    if trans_type == TransmissionType.manual:
      # get car's gearshift advice
#      if (0 < self.gearDesired < 7) and (0 < self.gearCurrent < 7):                     # 0 = gear not detected
#        self.gearAdvice = self.gearDesired - self.gearCurrent
#        self.gearAdviceValid = True
#      else:
#        self.gearAdvice = 0
#        self.gearAdviceValid = False

      # test RPM limit and prevent change as long as in hysteresis
      if self.engineRPM > 2500.0 and not self.gsaHystActive and ret.vEgo<120.*CV.KPH_TO_MS:
        self.gsaSpeedFreeze = ret.vEgo
        self.gsaHystActive = True
      # within hysteresis band -> set RPM intervention active
      if self.gsaHystActive:
        self.gsaIntvActive = True
      else:
        self.gsaIntvActive = False
      # handle hysteresis flag
      if self.engineRPM < 2200.0:   # or self.gearAdvice < 0
        self.gsaHystActive = False

      # assign desired values to generate desired set-speed (depending on driving situation)
      if ret.clutchPressed:                                           # during clutch open do not try to accelerate
        ret.cruiseState.speed = min(ret.vEgo, ret.cruiseState.speed)  # -> neutral speed setpoint but do not increase
                                                                      #    (to not prevent braking with clutch open)
      # apply limit when >RPM limit # + car advises to shift up
      # in last gear, no shift up advice is sent by ECU -> do not limit
      elif self.gsaIntvActive:      # and self.gearAdvice > 0  and self.gearAdviceValid     # >RPM limit + no shift up advice -> last gear
        ret.cruiseState.speed = self.gsaSpeedFreeze                   # limit RPM by using frozen speed

      ret.engineRPMlimited = self.gsaIntvActive

    if ret.cruiseState.speed > 70:  # 255 kph in m/s == no current setpoint
      ret.cruiseState.speed = 0


    ret.leftBlinker = bool(pt_cp.vl["Gate_Komf_1"]['GK1_Blinker_li'])
    ret.rightBlinker = bool(pt_cp.vl["Gate_Komf_1"]['GK1_Blinker_re'])

    # Read ACC hardware button type configuration info that has to pass thru
    # to the radar. Ends up being different for steering wheel buttons vs
    # third stalk type controls.
    # TODO: Check to see what info we need to passthru and spoof on PQ
    self.graHauptschalter = pt_cp.vl["GRA_Neu"]['GRA_Hauptschalt']
    self.graSenderCoding = pt_cp.vl["GRA_Neu"]['GRA_Sender']
    self.graTypHauptschalter = False
    self.graButtonTypeInfo = False
    self.graTipStufe2 = False
    # Pick up the GRA_ACC_01 CAN message counter so we can sync to it for
    # later cruise-control button spamming.
    # FIXME: will need msg counter and checksum algo to spoof GRA_neu
    self.graMsgBusCounter = pt_cp.vl["GRA_Neu"]['GRA_Neu_Zaehler']

    # Check to make sure the electric power steering rack is configured to
    # accept and respond to HCA_01 messages and has not encountered a fault.
    self.steeringFault = pt_cp.vl["Lenkhilfe_2"]['LH2_Sta_HCA'] not in [1, 3, 5, 7]

    # Read ABS pump for checking in ACC braking is working.
    if self.CP.enableGasInterceptor:
      self.ABSWorking = pt_cp.vl["Bremse_8"]["BR8_Sta_ADR_BR"]
      self.currentSpeed = ret.vEgo
    
    return ret
  
  @staticmethod
  def get_pq_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("LH3_BLW", "Lenkhilfe_3", 0),                # Absolute steering angle
      ("LH3_BLWSign", "Lenkhilfe_3", 0),            # Steering angle sign
      ("LH3_LM", "Lenkhilfe_3", 0),                 # Absolute driver torque input
      ("LH3_LMSign", "Lenkhilfe_3", 0),             # Driver torque input sign
      ("LH2_Sta_HCA", "Lenkhilfe_2", 0),            # Steering rack HCA status
      ("Lenkradwinkel_Geschwindigkeit", "Lenkwinkel_1", 0),  # Absolute steering rate
      ("Lenkradwinkel_Geschwindigkeit_S", "Lenkwinkel_1", 0),  # Steering rate sign
      ("Radgeschw__VL_4_1", "Bremse_3", 0),         # ABS wheel speed, front left
      ("Radgeschw__VR_4_1", "Bremse_3", 0),         # ABS wheel speed, front right
      ("Radgeschw__HL_4_1", "Bremse_3", 0),         # ABS wheel speed, rear left
      ("Radgeschw__HR_4_1", "Bremse_3", 0),         # ABS wheel speed, rear right
      ("Giergeschwindigkeit", "Bremse_5", 0),       # Absolute yaw rate
      ("Vorzeichen_der_Giergeschwindigk", "Bremse_5", 0),  # Yaw rate sign
      ("GK1_Fa_Tuerkont", "Gate_Komf_1", 0),     # Door open, driver
      # TODO: locate passenger and rear door states
      ("GK1_Blinker_li", "Gate_Komf_1", 0),         # Left turn signal on
      ("GK1_Blinker_re", "Gate_Komf_1", 0),         # Right turn signal on
      ("Gurtschalter_Fahrer", "Airbag_1", 0),       # Seatbelt status, driver
      ("Gurtschalter_Beifahrer", "Airbag_1", 0),    # Seatbelt status, passenger
      ("Bremstestschalter", "Motor_2", 0),          # Brake pedal pressed (brake light test switch)
      ("Bremslichtschalter", "Motor_2", 0),         # Brakes applied (brake light switch)
      ("Bremsdruck", "Bremse_5", 0),                # Brake pressure applied
      ("Vorzeichen_Bremsdruck", "Bremse_5", 0),     # Brake pressure applied sign (???)
      ("Fahrpedal_Rohsignal", "Motor_3", 0),        # Accelerator pedal value
      ("ESP_Passiv_getastet", "Bremse_1", 0),       # Stability control disabled
      ("MFA_v_Einheit_02", "Einheiten_1", 0),       # MPH vs KMH speed display
      ("Bremsinfo", "Kombi_1", 0),                  # Manual handbrake applied
      ("GRA_Status", "Motor_2", 0),                 # ACC engagement status
      ("Soll_Geschwindigkeit_bei_GRA_Be", "Motor_2", 0),  # ACC speed setpoint from ECU??? check this
      ("GRA_Hauptschalt", "GRA_Neu", 0),              # ACC button, on/off
      ("GRA_Abbrechen", "GRA_Neu", 0),                  # ACC button, cancel
      ("GRA_Neu_Setzen", "GRA_Neu", 0),                     # ACC button, set
      ("GRA_Up_lang", "GRA_Neu", 0),                # ACC button, increase or accel, long press
      ("GRA_Down_lang", "GRA_Neu", 0),              # ACC button, decrease or decel, long press
      ("GRA_Up_kurz", "GRA_Neu", 0),                # ACC button, increase or accel, short press
      ("GRA_Down_kurz", "GRA_Neu", 0),              # ACC button, decrease or decel, short press
      ("GRA_Recall", "GRA_Neu", 0),                 # ACC button, resume
      ("GRA_Zeitluecke", "GRA_Neu", 0),             # ACC button, time gap adj
      ("GRA_Neu_Zaehler", "GRA_Neu", 0),            # ACC button, time gap adj
      ("GRA_Sender", "GRA_Neu", 0),                 # GRA Sender Coding
      ("BR8_Sta_ADR_BR", "Bremse_8", 0),            # ABS Pump actively braking for ACC
      ("ESP_Eingriff", "Bremse_1", 0),              # ABS stability intervention
      ("ASR_Anforderung", "Bremse_1", 0),           # ABS slip detected      
    ]
    print("prepared messages")

    checks = [
      # sig_address, frequency
      ("Bremse_3", 100),          # From J104 ABS/ESP controller
      ("Lenkhilfe_3", 100),       # From J500 Steering Assist with integrated sensors
      ("Lenkwinkel_1", 100),      # From J500 Steering Assist with integrated sensors
      ("Motor_3", 100),           # From J623 Engine control module
      ("Airbag_1", 50),           # From J234 Airbag control module
      ("Bremse_5", 50),           # From J104 ABS/ESP controller
      ("Bremse_8", 50),           # From J??? ABS/ACC controller
      ("GRA_Neu", 50),            # From J??? steering wheel control buttons
      ("Kombi_1", 50),            # From J285 Instrument cluster
      ("Motor_2", 50),            # From J623 Engine control module
      ("Lenkhilfe_2", 20),        # From J500 Steering Assist with integrated sensors
      ("Gate_Komf_1", 10),        # From J533 CAN gateway
      ("Bremse_1", 10),
      ("Einheiten_1", 1),         # From J??? cluster or gateway
    ]
    print("prepared checks")

    signals += [("Waehlhebelposition__Getriebe_1_", "Getriebe_1", 0)]  # Auto trans gear selector position
    checks += [("Getriebe_1", 100)]  # From J743 Auto transmission control module

    print("prepared can paser fields")
    return CANParser(DBC_FILES.mqb, signals, checks, CANBUS.pt)

  @staticmethod
  def get_cam_can_parser(CP):
    print("getitng mqb cam car parseer")
    signals = []
    checks = []

    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        # sig_name, sig_address, default
        ("LDW_SW_Warnung_links", "LDW_02", 0),      # Blind spot in warning mode on left side due to lane departure
        ("LDW_SW_Warnung_rechts", "LDW_02", 0),     # Blind spot in warning mode on right side due to lane departure
        ("LDW_Seite_DLCTLC", "LDW_02", 0),          # Direction of most likely lane departure (left or right)
        ("LDW_DLC", "LDW_02", 0),                   # Lane departure, distance to line crossing
        ("LDW_TLC", "LDW_02", 0),                   # Lane departure, time to line crossing
      ]
      checks += [
        # sig_address, frequency
        ("LDW_02", 10)      # From R242 Driver assistance camera
      ]
    else:
      # Radars are here on CANBUS.cam
      signals += MqbExtraSignals.fwd_radar_signals
      checks += MqbExtraSignals.fwd_radar_checks
      if CP.enableBsm:
        signals += MqbExtraSignals.bsm_radar_signals
        checks += MqbExtraSignals.bsm_radar_checks

    return CANParser(DBC_FILES.mqb, signals, checks, CANBUS.cam)
  
  @staticmethod
  def get_pq_cam_can_parser(CP):
    print("getting pq cam car parser")
    # TODO: Need to monitor LKAS camera, if present, for TLC/DLC/warning signals for passthru to SWA
    signals = []
    checks = []

    if CP.networkLocation == NetworkLocation.gateway:
      # The ACC radar is here on CANBUS.cam
      signals += [("ACA_V_Wunsch", "ACC_GRA_Anziege", 0)]  # ACC set speed
      checks += [("ACC_GRA_Anziege", 25)]  # From J428 ACC radar control module

    if CP.enableGasInterceptor:
      signals += [("INTERCEPTOR_GAS", "GAS_SENSOR", 0), ("INTERCEPTOR_GAS2", "GAS_SENSOR", 0)]
      checks += [("GAS_SENSOR", 50)]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, CANBUS.cam)

class MqbExtraSignals:
  # Additional signal and message lists for optional or bus-portable controllers
  fwd_radar_signals = [
    ("ACC_Wunschgeschw", "ACC_02", 0),              # ACC set speed
    ("AWV2_Freigabe", "ACC_10", 0),                 # FCW brake jerk release
    ("ANB_Teilbremsung_Freigabe", "ACC_10", 0),     # AEB partial braking release
    ("ANB_Zielbremsung_Freigabe", "ACC_10", 0),     # AEB target braking release
  ]
  fwd_radar_checks = [
    ("ACC_10", 50),                                 # From J428 ACC radar control module
    ("ACC_02", 17),                                 # From J428 ACC radar control module
  ]
  bsm_radar_signals = [
    ("SWA_Infostufe_SWA_li", "SWA_01", 0),          # Blind spot object info, left
    ("SWA_Warnung_SWA_li", "SWA_01", 0),            # Blind spot object warning, left
    ("SWA_Infostufe_SWA_re", "SWA_01", 0),          # Blind spot object info, right
    ("SWA_Warnung_SWA_re", "SWA_01", 0),            # Blind spot object warning, right
  ]
  bsm_radar_checks = [
    ("SWA_01", 20),                                 # From J1086 Lane Change Assist
  ]
