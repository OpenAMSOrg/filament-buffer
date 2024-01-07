# Filament Buffer Pressure Sensor
#
# Copyright (C) 2023 JR Lomas (discord:knight_rad.iant) <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from .extruder_stepper import PrinterExtruderStepper

MAX_ROTATION_VARIANCE = 5.0 # given in mm

class FilamentPressureSensor:
    def __init__(self, config):
        #self._adc = config.get_printer().get_adc()
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self._pin = config.get('pin')
        self._sample_count = config.getint('sample_count', 5)
        self._sample_time = config.getfloat('sample_time', 0.005)
        self._report_time = config.getfloat('report_time', 0.050)

        self._pid_kp = config.getfloat('pid_kp', 5)
        self._pid_kd = config.getfloat('pid_kd', 3.0)
        self._pid_ki = config.getfloat('pid_ki', 0.5)

        self._sf_max_speed = config.getfloat('max_speed', 300.0)
        self._accel = config.getfloat('accel', 0.0)

        self._set_point = config.getfloat('set_point', 0.9)

        # printer objects
        self.ppins = self.adc = None
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        self.ppins = self.printer.lookup_object('pins')
        self.adc = self.ppins.setup_pin('adc', self._pin)
        self.adc.setup_minmax(self._sample_time, self._sample_count)
        self.adc.setup_adc_callback(self._report_time, self._adc_callback)

        #self.step_pin = self.printer.lookup_object('output_pin step_pin')

        self.filament_sensor_timer = self.reactor.register_timer(
            self.filament_sensor_timer_cb)

        self._pid = ControlPID(self, config)

        # Register commands
        self.gcode = self.printer.lookup_object('gcode')
        self.enable = False

        self.stepper = PrinterExtruderStepper(config)
        self.base_stepper = self.stepper.extruder_stepper.stepper

        self.base_rotation_distance = self.base_stepper.get_rotation_distance()[0]

        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('ENABLE_STEPPER_FOLLOWER', self.cmd_enable_stepper_follower)
        self.gcode.register_command('DISABLE_STEPPER_FOLLOWER', self.cmd_disable_stepper_follower)
        self.gcode.register_command('SET_STEPPER_FOLLOWER_PID', self.cmd_set_stepper_follower_pid)

    def cmd_set_stepper_follower_pid(self, gcmd):
        self._pid.Kp = gcmd.get_float('KP', self._pid_kp)
        self._pid.Kd = gcmd.get_float('KD', self._pid_kd)
        self._pid.Ki = gcmd.get_float('KI', self._pid_ki)
        #logging.info("PID values: %s, %s, %s", self._pid.Kp, self._pid.Kd, self._pid.Ki)
        self._pid.update_pid(self._pid.Kp, self._pid.Ki, self._pid.Kd)
        gcmd.respond_info("Stepper Follower PID Set")

    def cmd_enable_stepper_follower(self, gcmd):
        self.enable = True
        gcmd.respond_info("Stepper Follower Enabled")

    def cmd_disable_stepper_follower(self, gcmd):
        self.enable = False
        gcmd.respond_info("Stepper Follower Disabled")


    # Initialization
    def handle_ready(self):
        self.reactor.update_timer(self.filament_sensor_timer,
                                  self.reactor.NOW)

    def filament_sensor_timer_cb(self, eventtime):
        return eventtime + 1

    def _adc_callback(self, read_time, read_value):
        logging.info("read value: %s", read_value)
        control = self._pid.pressure_update(read_time, read_value, self._set_point)
        if self.enable:
            rotation_distance= self.base_rotation_distance - control * MAX_ROTATION_VARIANCE
            self.base_stepper.set_rotation_distance(rotation_distance)
            logging.info("Rotation Distance: %s", rotation_distance)

PID_PARAM_BASE = 255.0
PID_SETTLE_DELTA = 1.
PID_SETTLE_SLOPE = .1
MIN_DERIV_TIME = 0.1

# Pressure Sensor given in 0 <- no pressure to 1 <- max pressure
class ControlPID:
    def __init__(self, pressure_sensor, config):
        self.sensor = pressure_sensor

        self.Kp = config.getfloat('pid_Kp') #/ PID_PARAM_BASE
        self.Ki = config.getfloat('pid_Ki') #/ PID_PARAM_BASE
        self.Kd = config.getfloat('pid_Kd') #/ PID_PARAM_BASE

        self.min_deriv_time = MIN_DERIV_TIME #pressure_sensor.get_min_deriv_time()
        self.pressure_integ_max = 1.0

        self.prev_pressure = 0.0
        self.prev_pressure_time = 0.0
        self.prev_pressure_deriv = 0.0
        self.prev_pressure_integ = 0.0

    def update_pid(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def pressure_update(self, read_time, pressure, target_pressure):
        time_diff = read_time - self.prev_pressure_time
        # Calculate change of pressure
        pressure_diff = pressure - self.prev_pressure

        if time_diff >= self.min_deriv_time:
            pressure_deriv = pressure_diff / time_diff
        else:
            pressure_deriv = (self.prev_pressure_deriv * (self.min_deriv_time-time_diff)
                          + pressure_diff) / self.min_deriv_time

        # Calculate accumulated pressure "error"
        pressure_err = target_pressure - pressure
        pressure_integ = self.prev_pressure_integ + pressure_err * time_diff
        pressure_integ = max(-1.0, min(self.pressure_integ_max, pressure_integ))

        # Calculate output
        co = self.Kp*pressure_err + self.Ki*pressure_integ - self.Kd*pressure_deriv

        #logging.info("pid: %f@%.3f -> diff=%f deriv=%f err=%f integ=%f co=%f",
        #    pressure, read_time, pressure_diff, pressure_deriv, pressure_err, pressure_integ, co)

        bounded_co = max(-1.0, min(1, co))

        # Store state for next measurement
        self.prev_pressure = pressure
        self.prev_pressure_time = read_time
        self.prev_pressure_deriv = pressure_deriv
        if co == bounded_co:
            self.prev_pressure_integ = pressure_integ

        return bounded_co

def load_config_prefix(config):
    return FilamentPressureSensor(config)

