from opendbc.car import structs, get_safety_config, CanBusBase, Bus, create_button_events
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.gwm.carcontroller import CarController
from opendbc.car.gwm.carstate import CarState
from opendbc.car.gwm.values import GwmSafetyFlags

ButtonType = structs.CarState.ButtonEvent.Type
TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def __init__(self, CP, CP_SP=None):
    super().__init__(CP, CP_SP)
    self.CP_SP = CP_SP
    self.lat_active = False
    self.isEPSobeying = True
    self.steer_fault_temporary_counter = 0
    self.current_personality = 0
    self.pcm_follow_distance = 0
    self.press_gac_button = False

  def apply(self, CC, CC_SP, now_nanos):
    self.lat_active = CC.latActive
    hud_control = CC.hudControl
    self.current_personality = hud_control.leadDistanceBars
    return super().apply(CC, CC_SP, now_nanos)

  def update(self, CC, CC_SP, can_packets):
    cp = self.can_parsers[Bus.main]
    self.isEPSobeying = cp.vl["RX_STEER_RELATED"]["A_RX_STEER_REQUESTED"] == 1
    self.steer_fault_temporary_counter = (self.steer_fault_temporary_counter + 1) \
                                          if (self.lat_active and not self.isEPSobeying) else 0

    cp_cam = self.can_parsers[Bus.cam]
    self.pcm_follow_distance = cp_cam.vl["ACC"]["CAR_DISTANCE_SELECTION"]

    ret = super().update(CC, CC_SP, can_packets)
    ret.steerFaultTemporary |= self.steer_fault_temporary_counter > 100

    if (self.pcm_follow_distance == 4 and self.current_personality != 3) or \
       (self.pcm_follow_distance == 3 and self.current_personality != 3) or \
       (self.pcm_follow_distance == 2 and self.current_personality != 2) or \
       (self.pcm_follow_distance == 1 and self.current_personality != 1):
      self.press_gac_button = not self.press_gac_button
    ret.buttonEvents = create_button_events(self.press_gac_button, True, {1: ButtonType.gapAdjustCruise})

    return ret

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'gwm'

    cfgs = [get_safety_config(structs.CarParams.SafetyModel.gwm)]

    # If multipanda mapping is detected (offset >= 4), keep the first safety slot
    # as `noOutput` so an internal panda remains silent and the vehicle safety config
    # stays as the last entry (`-1`). This enables external panda to control the vehicle.
    CAN = CanBusBase(None, fingerprint)
    if CAN.offset >= 4:
      cfgs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))

    ret.safetyConfigs = cfgs

    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.4
    ret.steerAtStandstill = False

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[-1].safetyParam |= GwmSafetyFlags.LONG_CONTROL.value

      # Stop & Go: enabled so openpilot holds at standstill and resumes.
      # The Haval H6 GT has no native S&G — resume is handled in carcontroller
      # via an AP_ENABLE_COMMAND pulse when the planner wants to start moving.
      ret.autoResumeSng = True

      ret.longitudinalActuatorDelay = 0.25

      # vEgoStopping / vEgoStarting: speed thresholds (m/s) at which OP
      # transitions to/from the stopping state. Keep them tight so the car
      # actually holds position instead of creeping.
      ret.vEgoStopping = 0.25
      ret.vEgoStarting = 0.25

      # stopAccel: accel command sent while holding at standstill (m/s²).
      # More negative = firmer hold. -0.75 is conservative; tune down to
      # -1.0 if the car rolls on slopes.
      ret.stopAccel = -0.75

      # stoppingDecelRate: how fast OP ramps decel to stopAccel (m/s³).
      ret.stoppingDecelRate = 0.75

      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.4]

    return ret

  @staticmethod
  def _get_params_sp(ret: structs.CarParams, car_params_sp, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    # SunnyPilot-specific parameters extension.
    # For now, GWM has no SP-specific flags, but the hook must exist
    # so the SP framework can call it without AttributeError.
    return ret
