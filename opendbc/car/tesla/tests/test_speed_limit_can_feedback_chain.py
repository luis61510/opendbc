from types import SimpleNamespace

from opendbc.sunnypilot.car.tesla.carstate_ext import CarStateExt
from opendbc.sunnypilot.car.tesla.speed_limit_controller import TeslaSpeedLimitController
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP


IDLE_TEMPLATE = bytes.fromhex("2955000000000080")


def make_state(current_kph=27, target_kph=30, template_nanos=1_000_000_000):
  return SimpleNamespace(
    out=SimpleNamespace(
      cruiseState=SimpleNamespace(enabled=True, speedCluster=current_kph / 3.6),
      brakePressed=False,
    ),
    tesla_speed_limit_target=target_kph / 3.6,
    tesla_speed_limit_target_valid=True,
    tesla_speed_limit_assist_enabled=True,
    tesla_speed_button_template=IDLE_TEMPLATE,
    tesla_speed_button_template_nanos=template_nanos,
    tesla_speed_units="KPH",
    tesla_autopilot_active=False,
    tesla_manual_speed_adjustment_counter=0,
    tesla_speed_auto_resume_gesture_counter=0,
    _tesla_speed_resume_up_nanos=0,
    _tesla_speed_resume_down_nanos=0,
    _tesla_speed_resume_wait_idle=False,
  )


def make_control():
  return SimpleNamespace(enabled=True, cruiseControl=SimpleNamespace(cancel=False))


def test_complete_can_feedback_chain_steps_27_to_30_with_fresh_idle_templates():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = make_state()
  control = make_control()

  # The first valid target settles for 500 ms before the first wheel tick.
  assert controller.update(control, state, 1_000_000_000) == []
  first = controller.update(control, state, 1_500_000_000)
  assert len(first) == 1
  assert first[0].dat == bytes.fromhex("2955000100000080")

  # Tesla reports 28 km/h and the observed 0x3C2 stream returns to idle.
  state.out.cruiseState.speedCluster = 28 / 3.6
  CarStateExt.update_speed_button_template(state, IDLE_TEMPLATE, 2_000_000_000)
  second = controller.update(control, state, 2_000_000_000)
  assert len(second) == 1

  # Repeat for 29 km/h.
  state.out.cruiseState.speedCluster = 29 / 3.6
  CarStateExt.update_speed_button_template(state, IDLE_TEMPLATE, 2_500_000_000)
  third = controller.update(control, state, 2_500_000_000)
  assert len(third) == 1

  # The final feedback reaches the target; no fourth tick should be emitted.
  state.out.cruiseState.speedCluster = 30 / 3.6
  CarStateExt.update_speed_button_template(state, IDLE_TEMPLATE, 3_000_000_000)
  assert controller.update(control, state, 3_000_000_000) == []
  assert controller.current_display == 30
  assert controller.remaining_steps == 0


def test_feedback_without_a_fresh_idle_template_does_not_send_next_tick():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = make_state()
  control = make_control()

  controller.update(control, state, 1_000_000_000)
  assert len(controller.update(control, state, 1_500_000_000)) == 1

  # The displayed speed changes, so feedback is recognized, but no new idle
  # frame arrives. The old template has expired by the time the next tick is due.
  state.out.cruiseState.speedCluster = 28 / 3.6
  assert controller.update(control, state, 3_100_000_000) == []
  assert controller.pending_direction == 0
  assert controller.current_display == 28
