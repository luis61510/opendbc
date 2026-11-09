from types import SimpleNamespace

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


def test_controller_steps_27_to_30_when_feedback_and_idle_templates_are_observed():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = make_state()
  control = make_control()

  # The first valid target settles for 500 ms before the first wheel tick.
  assert controller.update(control, state, 1_000_000_000) == []
  first = controller.update(control, state, 1_500_000_000)
  assert len(first) == 1
  assert first[0].dat == bytes.fromhex("2955000100000080")

  # Simulate Tesla reporting 28 km/h and a fresh idle template being observed.
  state.out.cruiseState.speedCluster = 28 / 3.6
  state.tesla_speed_button_template = IDLE_TEMPLATE
  state.tesla_speed_button_template_nanos = 2_000_000_000
  second = controller.update(control, state, 2_000_000_000)
  assert len(second) == 1

  # Repeat for 29 km/h with another simulated idle-template update.
  state.out.cruiseState.speedCluster = 29 / 3.6
  state.tesla_speed_button_template = IDLE_TEMPLATE
  state.tesla_speed_button_template_nanos = 2_500_000_000
  third = controller.update(control, state, 2_500_000_000)
  assert len(third) == 1

  # The final feedback reaches the target; no fourth tick should be emitted.
  state.out.cruiseState.speedCluster = 30 / 3.6
  state.tesla_speed_button_template = IDLE_TEMPLATE
  state.tesla_speed_button_template_nanos = 3_000_000_000
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



def test_autopilot_ownership_blocks_synthetic_speed_wheel_ticks():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = make_state()
  state.tesla_autopilot_active = True
  control = make_control()

  # AP owns the wheel controls; even a valid target and fresh template cannot
  # cause SunnyPilot to synthesize a speed-wheel press.
  assert controller.update(control, state, 1_000_000_000) == []
  assert controller.update(control, state, 2_000_000_000) == []
  assert controller.pending_direction == 0


def test_manual_wheel_counter_pauses_controller_during_pending_feedback():
  controller = TeslaSpeedLimitController(SimpleNamespace(flags=TeslaFlagsSP.AUTO_SPEED_LIMIT))
  state = make_state()
  control = make_control()

  controller.update(control, state, 1_000_000_000)
  assert len(controller.update(control, state, 1_500_000_000)) == 1

  # A physical wheel event must win over a concurrent/late set-speed feedback.
  state.tesla_manual_speed_adjustment_counter += 1
  state.out.cruiseState.speedCluster = 28 / 3.6
  state.tesla_speed_button_template_nanos = 2_000_000_000
  assert controller.update(control, state, 2_000_000_000) == []
  assert controller.manual_override_active
  assert controller.pending_direction == 0
