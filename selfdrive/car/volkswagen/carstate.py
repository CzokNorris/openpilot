import numpy as np
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.volkswagen.values import DBC, DBC_FILES, CANBUS, TransmissionType, GearShifter, BUTTON_STATES, CarControllerParams

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])

    # Configure for PQ35/PQ46/NMS network messaging
    self.get_can_parser = self.get_pq_can_parser
    self.get_cam_can_parser = self.get_pq_cam_can_parser
    self.update = self.update_pq
    if CP.transmissionType == TransmissionType.automatic:
      self.shifter_values = can_define.dv["Getriebe_1"]['Waehlhebelposition__Getriebe_1_']
    #self.ok=False
    self.buttonStates = BUTTON_STATES.copy()

    
  def update_mqb(self, pt_cp, cam_cp, trans_type):
    ret = car.CarState.new_message()
    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds.fl = pt_cp.vl["ESP_19"]['ESP_VL_Radgeschw_02'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = pt_cp.vl["ESP_19"]['ESP_VR_Radgeschw_02'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = pt_cp.vl["ESP_19"]['ESP_HL_Radgeschw_02'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = pt_cp.vl["ESP_19"]['ESP_HR_Radgeschw_02'] * CV.KPH_TO_MS

    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.1

    # Update steering angle, rate, yaw rate, and driver input torque. VW send
    # the sign/direction in a separate signal so they must be recombined.
    ret.steeringAngleDeg = pt_cp.vl["LH_EPS_03"]['EPS_Berechneter_LW'] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]['EPS_VZ_BLW'])]
    ret.steeringRateDeg = pt_cp.vl["LWI_01"]['LWI_Lenkradw_Geschw'] * (1, -1)[int(pt_cp.vl["LWI_01"]['LWI_VZ_Lenkradw_Geschw'])]
    ret.steeringTorque = pt_cp.vl["LH_EPS_03"]['EPS_Lenkmoment'] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]['EPS_VZ_Lenkmoment'])]
    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["ESP_02"]['ESP_Gierrate'] * (1, -1)[int(pt_cp.vl["ESP_02"]['ESP_VZ_Gierrate'])] * CV.DEG_TO_RAD

    # Update gas, brakes, and gearshift.
    ret.gas = pt_cp.vl["Motor_20"]['MO_Fahrpedalrohwert_01'] / 100.0
    ret.gasPressed = ret.gas > 0
    ret.brake = pt_cp.vl["ESP_05"]['ESP_Bremsdruck'] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["ESP_05"]['ESP_Fahrer_bremst'])
    ret.brakeLights = bool(pt_cp.vl["ESP_05"]['ESP_Status_Bremsdruck'])
    ret.gearShifter = GearShifter.drive

    # Update door and trunk/hatch lid open status.
    ret.doorOpen = any([pt_cp.vl["Gateway_72"]['ZV_FT_offen'],
                        pt_cp.vl["Gateway_72"]['ZV_BT_offen'],
                        pt_cp.vl["Gateway_72"]['ZV_HFS_offen'],
                        pt_cp.vl["Gateway_72"]['ZV_HBFS_offen'],
                        pt_cp.vl["Gateway_72"]['ZV_HD_offen']])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = pt_cp.vl["Airbag_02"]["AB_Gurtschloss_FA"] != 3

    # Update driver preference for metric. VW stores many different unit
    # preferences, including separate units for for distance vs. speed.
    # We use the speed preference for OP.
    self.displayMetricUnits = not pt_cp.vl["Einheiten_01"]["KBI_MFA_v_Einheit_02"]

    # Consume blind-spot monitoring info/warning LED states, if available. The
    # info signal (LED on) is enabled whenever a vehicle is detected in the
    # driver's blind spot. The warning signal (LED flashing) is enabled if the
    # driver shows possibly hazardous intent toward a BSM detected vehicle, by
    # setting the turn signal in that direction, or (for cars with factory Lane
    # Assist) approaches the lane boundary in that direction. Size of the BSM
    # detection box is dynamic based on speed and road curvature.
    # Refer to VW Self Study Program 890253: Volkswagen Driver Assist Systems,
    # pages 32-35.
    ret.leftBlindspot = bool(pt_cp.vl["SWA_01"]["SWA_Infostufe_SWA_li"]) or bool(pt_cp.vl["SWA_01"]["SWA_Warnung_SWA_li"])
    ret.rightBlindspot = bool(pt_cp.vl["SWA_01"]["SWA_Infostufe_SWA_re"]) or bool(pt_cp.vl["SWA_01"]["SWA_Warnung_SWA_re"])

    # Consume factory LDW data relevant for factory SWA (Lane Change Assist)
    # and capture it for forwarding to the blind spot radar controller
    self.ldw_lane_warning_left = bool(cam_cp.vl["LDW_02"]["LDW_SW_Warnung_links"])
    self.ldw_lane_warning_right = bool(cam_cp.vl["LDW_02"]["LDW_SW_Warnung_rechts"])
    self.ldw_side_dlc_tlc = bool(cam_cp.vl["LDW_02"]["LDW_Seite_DLCTLC"])
    self.ldw_dlc = cam_cp.vl["LDW_02"]["LDW_DLC"]
    self.ldw_tlc = cam_cp.vl["LDW_02"]["LDW_TLC"]

    # Stock FCW is considered active if the release bit for brake-jerk warning
    # is set. Stock AEB considered active if the partial braking or target
    # braking release bits are set.
    # Refer to VW Self Study Program 890253: Volkswagen Driver Assistance
    # Systems, chapter on Front Assist with Braking: Golf Family for all MQB
    ret.stockFcw = bool(ext_cp.vl["ACC_10"]["AWV2_Freigabe"])
    ret.stockAeb = bool(ext_cp.vl["ACC_10"]["ANB_Teilbremsung_Freigabe"]) or bool(ext_cp.vl["ACC_10"]["ANB_Zielbremsung_Freigabe"])

    # Update ACC radar status.
    accStatus = pt_cp.vl["TSK_06"]['TSK_Status']
    if accStatus == 2:
      # ACC okay and enabled, but not currently engaged
      ret.cruiseState.available = True
      ret.cruiseState.enabled = False
    elif accStatus in [3, 4, 5]:
      # ACC okay and enabled, currently engaged and regulating speed (3) or engaged with driver accelerating (4) or overrun (5)
      ret.cruiseState.available = True
      ret.cruiseState.enabled = True
    else:
      # ACC okay but disabled (1), or a radar visibility or other fault/disruption (6 or 7)
      ret.cruiseState.available = False
      ret.cruiseState.enabled = False

    # Update ACC setpoint. When the setpoint is zero or there's an error, the
    # radar sends a set-speed of ~90.69 m/s / 203mph.
    ret.cruiseState.speed = pt_cp.vl["ACC_02"]['ACC_Wunschgeschw'] * CV.KPH_TO_MS
    if ret.cruiseState.speed > 90:
      ret.cruiseState.speed = 0

    # Update control button states for turn signals and ACC controls.
    self.buttonStates["accelCruise"] = bool(pt_cp.vl["GRA_ACC_01"]['GRA_Tip_Hoch'])
    self.buttonStates["decelCruise"] = bool(pt_cp.vl["GRA_ACC_01"]['GRA_Tip_Runter'])
    self.buttonStates["cancel"] = bool(pt_cp.vl["GRA_ACC_01"]['GRA_Abbrechen'])
    self.buttonStates["setCruise"] = bool(pt_cp.vl["GRA_ACC_01"]['GRA_Tip_Setzen'])
    self.buttonStates["resumeCruise"] = bool(pt_cp.vl["GRA_ACC_01"]['GRA_Tip_Wiederaufnahme'])
    self.buttonStates["gapAdjustCruise"] = bool(pt_cp.vl["GRA_ACC_01"]['GRA_Verstellung_Zeitluecke'])
    ret.leftBlinker = bool(pt_cp.vl["Gateway_72"]['BH_Blinker_li'])
    ret.rightBlinker = bool(pt_cp.vl["Gateway_72"]['BH_Blinker_re'])

    # Read ACC hardware button type configuration info that has to pass thru
    # to the radar. Ends up being different for steering wheel buttons vs
    # third stalk type controls.
    self.graHauptschalter = pt_cp.vl["GRA_ACC_01"]['GRA_Hauptschalter']
    self.graTypHauptschalter = pt_cp.vl["GRA_ACC_01"]['GRA_Typ_Hauptschalter']
    self.graButtonTypeInfo = pt_cp.vl["GRA_ACC_01"]['GRA_ButtonTypeInfo']
    self.graTipStufe2 = pt_cp.vl["GRA_ACC_01"]['GRA_Tip_Stufe_2']
    # Pick up the GRA_ACC_01 CAN message counter so we can sync to it for
    # later cruise-control button spamming.
    self.graMsgBusCounter = pt_cp.vl["GRA_ACC_01"]['COUNTER']

    # Check to make sure the electric power steering rack is configured to
    # accept and respond to HCA_01 messages and has not encountered a fault.
    self.steeringFault = not pt_cp.vl["LH_EPS_03"]["EPS_HCA_Status"]

    # Additional safety checks performed in CarInterface.
    self.parkingBrakeSet = bool(pt_cp.vl["Kombi_01"]['KBI_Handbremse'])  # FIXME: need to include an EPB check as well
    ret.espDisabled = pt_cp.vl["ESP_21"]['ESP_Tastung_passiv'] != 0

    return ret

  def update_pq(self, pt_cp, cam_cp, trans_type):
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
    ret.steeringAngleDeg = pt_cp.vl["Lenkhilfe_3"]['LH3_BLW'] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]['LH3_BLWSign'])]
    ret.steeringRateDeg = pt_cp.vl["Lenkwinkel_1"]['Lenkradwinkel_Geschwindigkeit'] * (1, -1)[int(pt_cp.vl["Lenkwinkel_1"]['Lenkradwinkel_Geschwindigkeit_S'])]
    ret.steeringTorque = pt_cp.vl["Lenkhilfe_3"]['LH3_LM'] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]['LH3_LMSign'])]
    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["Bremse_5"]['Giergeschwindigkeit'] * (1, -1)[int(pt_cp.vl["Bremse_5"]['Vorzeichen_der_Giergeschwindigk'])] * CV.DEG_TO_RAD

    # Update gas, brakes, and gearshift.
    ret.gas = 0
    ret.gasPressed = ret.gas > 0
    ret.brake = pt_cp.vl["Bremse_5"]['Bremsdruck'] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["Motor_2"]['Bremstestschalter'])
    ret.gearShifter = GearShifter.drive

    # Update door and trunk/hatch lid open status.
    # TODO: need to locate signals for other three doors if possible
    ret.doorOpen = bool(pt_cp.vl["Gate_Komf_1"]['GK1_Fa_Tuerkont'])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = False

    # Update driver preference for metric. VW stores many different unit
    # preferences, including separate units for for distance vs. speed.
    # We use the speed preference for OP.
    self.displayMetricUnits = not pt_cp.vl["Einheiten_1"]["MFA_v_Einheit_02"]

    # TODO: Consume blind spot data from factory radar, if present
    # TODO: Consume lane departure data from factory camera, if present
    self.ldw_lane_warning_left = False
    self.ldw_lane_warning_right = False
    self.ldw_side_dlc_tlc = None
    self.ldw_dlc = None
    self.ldw_tlc = None
    # TODO: Consume FCW/AEB data from factory radar, if present

    # Update ACC radar status.
    ret.cruiseState.available = bool(pt_cp.vl["GRA_Neu"]['GRA_Hauptschalt'])
    ret.cruiseState.enabled = True if pt_cp.vl["Motor_2"]['GRA_Status'] in [1, 2] else False

    # Update ACC setpoint. When the setpoint reads as 255, the driver has not
    # yet established an ACC setpoint, so treat it as zero.
    # FIXME: for now this is the CC setpoint, the radar setpoint is in ACC_GRA_Anziege.ACA_V_Wunsch
    ret.cruiseState.speed = pt_cp.vl["Motor_2"]['Soll_Geschwindigkeit_bei_GRA_Be'] * CV.KPH_TO_MS
    if ret.cruiseState.speed > 70:  # 255 kph in m/s == no current setpoint
      ret.cruiseState.speed = 0

    # Update control button states for turn signals and ACC controls.
    self.buttonStates["accelCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Up_kurz']) or bool(pt_cp.vl["GRA_Neu"]['GRA_Up_lang'])
    self.buttonStates["decelCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Down_kurz']) or bool(pt_cp.vl["GRA_Neu"]['GRA_Down_lang'])
    self.buttonStates["cancel"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Abbrechen'])
    self.buttonStates["setCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Neu_Setzen'])
    self.buttonStates["resumeCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Recall'])
    self.buttonStates["gapAdjustCruise"] = bool(pt_cp.vl["GRA_Neu"]['GRA_Zeitluecke'])
    ret.leftBlinker = bool(pt_cp.vl["Gate_Komf_1"]['GK1_Blinker_li'])
    ret.rightBlinker = bool(pt_cp.vl["Gate_Komf_1"]['GK1_Blinker_re'])

    #self.leftBlinker = ret.leftBlinker and self.ok
    #self.rightBlinker = ret.rightBlinker and self.ok


    # Read ACC hardware button type configuration info that has to pass thru
    # to the radar. Ends up being different for steering wheel buttons vs
    # third stalk type controls.
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

    # Additional safety checks performed in CarInterface.
    self.parkingBrakeSet = bool(pt_cp.vl["Kombi_1"]['Bremsinfo'])  # FIXME: need to include an EPB check as well
    ret.espDisabled = bool(pt_cp.vl["Bremse_1"]['ESP_Passiv_getastet'])

    #self.ok |= ret.espDisabled
    return ret

  @staticmethod
  def get_mqb_can_parser(CP):
    # this function generates lists for signal, messages and initial values
    signals = [
      # sig_name, sig_address, default
      ("EPS_Berechneter_LW", "LH_EPS_03", 0),       # Absolute steering angle
      ("EPS_VZ_BLW", "LH_EPS_03", 0),               # Steering angle sign
      ("LWI_Lenkradw_Geschw", "LWI_01", 0),         # Absolute steering rate
      ("LWI_VZ_Lenkradw_Geschw", "LWI_01", 0),      # Steering rate sign
      ("ESP_VL_Radgeschw_02", "ESP_19", 0),         # ABS wheel speed, front left
      ("ESP_VR_Radgeschw_02", "ESP_19", 0),         # ABS wheel speed, front right
      ("ESP_HL_Radgeschw_02", "ESP_19", 0),         # ABS wheel speed, rear left
      ("ESP_HR_Radgeschw_02", "ESP_19", 0),         # ABS wheel speed, rear right
      ("ESP_Gierrate", "ESP_02", 0),                # Absolute yaw rate
      ("ESP_VZ_Gierrate", "ESP_02", 0),             # Yaw rate sign
      ("ZV_FT_offen", "Gateway_72", 0),             # Door open, driver
      ("ZV_BT_offen", "Gateway_72", 0),             # Door open, passenger
      ("ZV_HFS_offen", "Gateway_72", 0),            # Door open, rear left
      ("ZV_HBFS_offen", "Gateway_72", 0),           # Door open, rear right
      ("ZV_HD_offen", "Gateway_72", 0),             # Trunk or hatch open
      ("Comfort_Signal_Left", "Blinkmodi_02", 0),   # Left turn signal including comfort blink interval
      ("Comfort_Signal_Right", "Blinkmodi_02", 0),  # Right turn signal including comfort blink interval
      ("AB_Gurtschloss_FA", "Airbag_02", 0),        # Seatbelt status, driver
      ("AB_Gurtschloss_BF", "Airbag_02", 0),        # Seatbelt status, passenger
      ("ESP_Fahrer_bremst", "ESP_05", 0),           # Brake pedal pressed
      ("ESP_Status_Bremsdruck", "ESP_05", 0),       # Brakes applied
      ("ESP_Bremsdruck", "ESP_05", 0),              # Brake pressure applied
      ("MO_Fahrpedalrohwert_01", "Motor_20", 0),    # Accelerator pedal value
      ("EPS_Lenkmoment", "LH_EPS_03", 0),           # Absolute driver torque input
      ("EPS_VZ_Lenkmoment", "LH_EPS_03", 0),        # Driver torque input sign
      ("EPS_HCA_Status", "LH_EPS_03", 0),           # Steering rack HCA support configured
      ("ESP_Tastung_passiv", "ESP_21", 0),          # Stability control disabled
      ("KBI_MFA_v_Einheit_02", "Einheiten_01", 0),  # MPH vs KMH speed display
      ("KBI_Handbremse", "Kombi_01", 0),            # Manual handbrake applied
      ("TSK_Status", "TSK_06", 0),                  # ACC engagement status from drivetrain coordinator
      ("TSK_Fahrzeugmasse_02", "Motor_16", 0),      # Estimated vehicle mass from drivetrain coordinator
      ("ACC_Wunschgeschw", "ACC_02", 0),            # ACC set speed
      ("AWV2_Freigabe", "ACC_10", 0),               # FCW brake jerk release
      ("ANB_Teilbremsung_Freigabe", "ACC_10", 0),   # AEB partial braking release
      ("ANB_Zielbremsung_Freigabe", "ACC_10", 0),   # AEB target braking release
      ("GRA_Hauptschalter", "GRA_ACC_01", 0),       # ACC button, on/off
      ("GRA_Abbrechen", "GRA_ACC_01", 0),           # ACC button, cancel
      ("GRA_Tip_Setzen", "GRA_ACC_01", 0),          # ACC button, set
      ("GRA_Tip_Hoch", "GRA_ACC_01", 0),            # ACC button, increase or accel
      ("GRA_Tip_Runter", "GRA_ACC_01", 0),          # ACC button, decrease or decel
      ("GRA_Tip_Wiederaufnahme", "GRA_ACC_01", 0),  # ACC button, resume
      ("GRA_Verstellung_Zeitluecke", "GRA_ACC_01", 0),  # ACC button, time gap adj
      ("GRA_Typ_Hauptschalter", "GRA_ACC_01", 0),   # ACC main button type
      ("GRA_Tip_Stufe_2", "GRA_ACC_01", 0),         # unknown related to stalk type
      ("GRA_ButtonTypeInfo", "GRA_ACC_01", 0),      # unknown related to stalk type
      ("COUNTER", "GRA_ACC_01", 0),                 # GRA_ACC_01 CAN message counter
      ("SWA_Infostufe_SWA_li", "SWA_01", 0),        # Blind spot object info, left
      ("SWA_Warnung_SWA_li", "SWA_01", 0),          # Blind spot object warning, left
      ("SWA_Infostufe_SWA_re", "SWA_01", 0),        # Blind spot object info, right
      ("SWA_Warnung_SWA_re", "SWA_01", 0),          # Blind spot object warning, right
    ]

    checks = [
      # sig_address, frequency
      ("LWI_01", 100),      # From J500 Steering Assist with integrated sensors
      ("LH_EPS_03", 100),   # From J500 Steering Assist with integrated sensors
      ("ESP_19", 100),      # From J104 ABS/ESP controller
      ("ESP_05", 50),       # From J104 ABS/ESP controller
      ("ESP_21", 50),       # From J104 ABS/ESP controller
      ("ACC_10", 50),       # From J428 ACC radar control module
      ("Motor_20", 50),     # From J623 Engine control module
      ("TSK_06", 50),       # From J623 Engine control module
      ("GRA_ACC_01", 33),   # From J??? steering wheel control buttons
      ("ACC_02", 17),       # From J428 ACC radar control module
      ("Gateway_72", 10),   # From J533 CAN gateway (aggregated data)
      ("Airbag_02", 5),     # From J234 Airbag control module
      ("Kombi_01", 2),      # From J285 Instrument cluster
      ("Motor_16", 2),      # From J623 Engine control module
      ("Einheiten_01", 1),  # From J??? not known if gateway, cluster, or BCM
    ]

    if CP.transmissionType == TransmissionType.automatic:
      signals += [("GE_Fahrstufe", "Getriebe_11", 0)]  # Auto trans gear selector position
      checks += [("Getriebe_11", 20)]  # From J743 Auto transmission control module
    elif CP.transmissionType == TransmissionType.direct:
      signals += [("GearPosition", "EV_Gearshift", 0)]  # EV gear selector position
      checks += [("EV_Gearshift", 10)]  # From J??? unknown EV control module
    elif CP.transmissionType == TransmissionType.manual:
      signals += [("MO_Kuppl_schalter", "Motor_14", 0),  # Clutch switch
                  ("BCM1_Rueckfahrlicht_Schalter", "Gateway_72", 0)]  # Reverse light from BCM
      checks += [("Motor_14", 10)]  # From J623 Engine control module

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, CANBUS.pt)

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
      ("GK1_Fa_Tuerkont", "Gate_Komf_1", 0),        # Door open, driver
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
      ("Soll_Geschwindigkeit_bei_GRA_Be", "Motor_2", 0),  # FIXME: CC setpoint from ECU, does this work for ACC?
      ("GRA_Hauptschalt", "GRA_Neu", 0),            # ACC button, on/off
      ("GRA_Abbrechen", "GRA_Neu", 0),              # ACC button, cancel
      ("GRA_Neu_Setzen", "GRA_Neu", 0),             # ACC button, set
      ("GRA_Up_lang", "GRA_Neu", 0),                # ACC button, increase or accel, long press
      ("GRA_Down_lang", "GRA_Neu", 0),              # ACC button, decrease or decel, long press
      ("GRA_Up_kurz", "GRA_Neu", 0),                # ACC button, increase or accel, short press
      ("GRA_Down_kurz", "GRA_Neu", 0),              # ACC button, decrease or decel, short press
      ("GRA_Recall", "GRA_Neu", 0),                 # ACC button, resume
      ("GRA_Zeitluecke", "GRA_Neu", 0),             # ACC button, time gap adj
      ("GRA_Neu_Zaehler", "GRA_Neu", 0),            # ACC button, time gap adj
      ("GRA_Sender", "GRA_Neu", 0),                 # GRA Sender Coding
    ]

    checks = [
      # sig_address, frequency
      # ("Bremse_3", 100),          # From J104 ABS/ESP controller
      # ("Lenkhilfe_3", 100),       # From J500 Steering Assist with integrated sensors
      # ("Lenkwinkel_1", 100),      # From J500 Steering Assist with integrated sensors
      # ("Motor_3", 100),           # From J623 Engine control module
      # ("Airbag_1", 50),           # From J234 Airbag control module
      # ("Bremse_5", 50),           # From J104 ABS/ESP controller
      # ("GRA_Neu", 50),            # From J??? steering wheel control buttons
      # ("Kombi_1", 50),            # From J285 Instrument cluster
      # ("Motor_2", 50),            # From J623 Engine control module
      # ("Lenkhilfe_2", 20),        # From J500 Steering Assist with integrated sensors
      # ("Gate_Komf_1", 10),        # From J533 CAN gateway
      # ("Einheiten_1", 1),         # From J??? cluster or gateway
    ]

    if CP.transmissionType == TransmissionType.automatic:
      signals += [("Waehlhebelposition__Getriebe_1_", "Getriebe_1", 0)]  # Auto trans gear selector position
      checks += [("Getriebe_1", 100)]  # From J743 Auto transmission control module
    elif CP.transmissionType == TransmissionType.manual:
      signals += [("Kupplungsschalter", "Motor_1", 0),  # Clutch switch
                  ("GK1_Rueckfahr", "Gate_Komf_1", 0)]  # Reverse light from BCM
      checks += [("Motor_1", 100)]  # From J623 Engine control module

    # FIXME: ignoring ACC signals until we detect its presence and bus location
    #   signals += [("ACA_V_Wunsch", "ACC_GRA_Anziege", 0)]  # ACC set speed
    #   checks += [("ACC_GRA_Anziege", 25)]  # From J428 ACC radar control module

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, CANBUS.pt, False)

  @staticmethod
  def get_mqb_cam_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("LDW_SW_Warnung_links", "LDW_02", 0),          # Blind spot in warning mode on left side due to lane departure
      ("LDW_SW_Warnung_rechts", "LDW_02", 0),         # Blind spot in warning mode on right side due to lane departure
      ("LDW_Seite_DLCTLC", "LDW_02", 0),              # Direction of most likely lane departure (left or right)
      ("LDW_DLC", "LDW_02", 0),                       # Lane departure, distance to line crossing
      ("LDW_TLC", "LDW_02", 0),                       # Lane departure, time to line crossing
    ]

    checks = [
      # sig_address, frequency
      ("LDW_02", 10)        # From R242 Driver assistance camera
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, CANBUS.cam)

  @staticmethod
  def get_pq_cam_can_parser(CP):
    # TODO: Need to monitor LKAS camera, if present, for TLC/DLC/warning signals for passthru to SWA
    signals = []
    checks = []

    # FIXME: ignoring ACC signals until we detect its presence and bus location
    #   signals += [("ACA_V_Wunsch", "ACC_GRA_Anziege", 0)]  # ACC set speed
    #   checks += [("ACC_GRA_Anziege", 25)]  # From J428 ACC radar control module

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, CANBUS.cam)
