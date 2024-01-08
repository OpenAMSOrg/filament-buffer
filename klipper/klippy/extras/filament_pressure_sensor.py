# Filament Buffer Pressure Sensor
#
# Copyright (C) 2023 - 2024 JR Lomas (discord:knight_rad.iant) <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from .extruder_stepper import PrinterExtruderStepper

MAX_ROTATION_VARIANCE = 5.0 # given in mm
PIN_MIN_TIME = 0.01
RESEND_HOST_TIME = 0.05 + PIN_MIN_TIME
MAX_SCHEDULE_TIME = 0.1

class RTOutputPin:
    def __init__(self, config, pin_name=None, pin_address=None):
        if pin_name is None:
            pin_name = config.get('pin')
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object('pins')
        self.is_pwm = config.getboolean('pwm', False)
        self.cycle_time = None
        if self.is_pwm:
            self.mcu_pin = ppins.setup_pin('pwm', pin_name)
            cycle_time = config.getfloat('cycle_time', 0.100, above=0.,
                                         maxval=MAX_SCHEDULE_TIME)
            self.cycle_time = cycle_time
            hardware_pwm = config.getboolean('hardware_pwm', False)
            self.mcu_pin.setup_cycle_time(cycle_time, hardware_pwm)
            self.scale = config.getfloat('scale', 1., above=0.)
            self.last_cycle_time = self.default_cycle_time = cycle_time
        else:
            self.mcu_pin = ppins.setup_pin('digital_out', pin_name)
            self.scale = 1.
            self.last_cycle_time = self.default_cycle_time = 0.
        self.last_print_time = 0.
        static_value = config.getfloat('static_value', None,
                                       minval=0., maxval=self.scale)
        self.reactor = self.printer.get_reactor()
        self.resend_timer = None
        self.resend_interval = 0.
        if static_value is not None:
            self.mcu_pin.setup_max_duration(0.)
            self.last_value = static_value / self.scale
            self.mcu_pin.setup_start_value(
                self.last_value, self.last_value, True)
        else:
            max_mcu_duration = config.getfloat('maximum_mcu_duration', 0.,
                                               minval=0.500,
                                               maxval=MAX_SCHEDULE_TIME)
            self.mcu_pin.setup_max_duration(max_mcu_duration)
            if max_mcu_duration:
                self.resend_interval = max_mcu_duration - RESEND_HOST_TIME

            self.last_value = config.getfloat(
                'value', 0., minval=0., maxval=self.scale) / self.scale
            self.shutdown_value = config.getfloat(
                'shutdown_value', 0., minval=0., maxval=self.scale) / self.scale
            self.mcu_pin.setup_start_value(self.last_value, self.shutdown_value)
            pin_name = config.get_name().split()[1]
            if pin_address is not None:
                pin_name = pin_name + "_" + pin_address
            gcode = self.printer.lookup_object('gcode')
            gcode.register_mux_command("SET_PIN", "PIN", pin_name,
                                       self.cmd_SET_PIN,
                                       desc=self.cmd_SET_PIN_help)
    def get_status(self, eventtime):
        return {'value': self.last_value}
    def _set_pin(self, print_time, value, cycle_time, is_resend=False):
        if value == self.last_value and cycle_time == self.last_cycle_time:
            if not is_resend:
                return
        print_time = max(print_time, self.last_print_time + PIN_MIN_TIME)
        if self.is_pwm:
            self.mcu_pin.set_pwm(print_time, value, cycle_time)
        else:
            self.mcu_pin.set_digital(print_time, value)
        self.last_value = value
        self.last_cycle_time = cycle_time
        self.last_print_time = print_time
        if self.resend_interval and self.resend_timer is None:
            self.resend_timer = self.reactor.register_timer(
                self._resend_current_val, self.reactor.NOW)
    cmd_SET_PIN_help = "Set the value of an output pin"
    def cmd_SET_PIN(self, gcmd):
        value = gcmd.get_float('VALUE', minval=0., maxval=self.scale)
        value /= self.scale
        cycle_time = gcmd.get_float('CYCLE_TIME', self.default_cycle_time,
                                    above=0., maxval=MAX_SCHEDULE_TIME)
        if not self.is_pwm and value not in [0., 1.]:
            raise gcmd.error("Invalid pin value")
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback(
            lambda print_time: self._set_pin(print_time, value, cycle_time))
        
    def set_pin(self, value, cycle_time = None):
        if cycle_time is None:
            cycle_time = self.default_cycle_time
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback(
            lambda print_time: self._set_pin(print_time, value, cycle_time))

    def _resend_current_val(self, eventtime):
        if self.last_value == self.shutdown_value:
            self.reactor.unregister_timer(self.resend_timer)
            self.resend_timer = None
            return self.reactor.NEVER

        systime = self.reactor.monotonic()
        print_time = self.mcu_pin.get_mcu().estimated_print_time(systime)
        time_diff = (self.last_print_time + self.resend_interval) - print_time
        if time_diff > 0.:
            # Reschedule for resend time
            return systime + time_diff
        self._set_pin(print_time + PIN_MIN_TIME,
                      self.last_value, self.last_cycle_time, True)
        return systime + self.resend_interval


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
        
        self.hes_gauge_pins = []
        pin_names = config.get('hes_gauge_pins',None)
        if pin_names is not None:
            i = 0
            for pin_name in pin_names.split(","):
                self.hes_gauge_pins.append(RTOutputPin(config, pin_name, str(i)))
                i += 1

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
        #logging.info("read value: %s", read_value)
        n_pins = len(self.hes_gauge_pins)
        for i in range(0, n_pins):
            if read_value > (i+1)/n_pins:
                value = 1.0
            elif read_value > i/n_pins and read_value <= (i+1)/n_pins:
                value = (read_value - i/n_pins) / (1/n_pins)
            else:
                value = 0.0
            #logging.info("%i:%f" % (i,value))
            
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.register_lookahead_callback(
                lambda print_time: self.hes_gauge_pins[i]._set_pin(print_time, value,
                            self.hes_gauge_pins[i].cycle_time))
        
        
        control = self._pid.pressure_update(read_time, read_value, self._set_point)
        if self.enable:
            rotation_distance= self.base_rotation_distance - control * MAX_ROTATION_VARIANCE
            self.base_stepper.set_rotation_distance(rotation_distance)
            #logging.info("Rotation Distance: %s", rotation_distance)

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
# Filament Buffer Pressure Sensor
#
# Copyright (C) 2023 JR Lomas (discord:knight_rad.iant) <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from .extruder_stepper import PrinterExtruderStepper

MAX_ROTATION_VARIANCE = 5.0 # given in mm
PIN_MIN_TIME = 0.01
RESEND_HOST_TIME = 0.05 + PIN_MIN_TIME
MAX_SCHEDULE_TIME = 0.1

class RTOutputPin:
    def __init__(self, config, pin_name=None, pin_address=None):
        if pin_name is None:
            pin_name = config.get('pin')
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object('pins')
        self.is_pwm = config.getboolean('pwm', False)
        self.cycle_time = None
        if self.is_pwm:
            self.mcu_pin = ppins.setup_pin('pwm', pin_name)
            cycle_time = config.getfloat('cycle_time', 0.100, above=0.,
                                         maxval=MAX_SCHEDULE_TIME)
            self.cycle_time = cycle_time
            hardware_pwm = config.getboolean('hardware_pwm', False)
            self.mcu_pin.setup_cycle_time(cycle_time, hardware_pwm)
            self.scale = config.getfloat('scale', 1., above=0.)
            self.last_cycle_time = self.default_cycle_time = cycle_time
        else:
            self.mcu_pin = ppins.setup_pin('digital_out', pin_name)
            self.scale = 1.
            self.last_cycle_time = self.default_cycle_time = 0.
        self.last_print_time = 0.
        static_value = config.getfloat('static_value', None,
                                       minval=0., maxval=self.scale)
        self.reactor = self.printer.get_reactor()
        self.resend_timer = None
        self.resend_interval = 0.
        if static_value is not None:
            self.mcu_pin.setup_max_duration(0.)
            self.last_value = static_value / self.scale
            self.mcu_pin.setup_start_value(
                self.last_value, self.last_value, True)
        else:
            max_mcu_duration = config.getfloat('maximum_mcu_duration', 0.,
                                               minval=0.500,
                                               maxval=MAX_SCHEDULE_TIME)
            self.mcu_pin.setup_max_duration(max_mcu_duration)
            if max_mcu_duration:
                self.resend_interval = max_mcu_duration - RESEND_HOST_TIME

            self.last_value = config.getfloat(
                'value', 0., minval=0., maxval=self.scale) / self.scale
            self.shutdown_value = config.getfloat(
                'shutdown_value', 0., minval=0., maxval=self.scale) / self.scale
            self.mcu_pin.setup_start_value(self.last_value, self.shutdown_value)
            pin_name = config.get_name().split()[1]
            if pin_address is not None:
                pin_name = pin_name + "_" + pin_address
            gcode = self.printer.lookup_object('gcode')
            gcode.register_mux_command("SET_PIN", "PIN", pin_name,
                                       self.cmd_SET_PIN,
                                       desc=self.cmd_SET_PIN_help)
    def get_status(self, eventtime):
        return {'value': self.last_value}
    def _set_pin(self, print_time, value, cycle_time, is_resend=False):
        if value == self.last_value and cycle_time == self.last_cycle_time:
            if not is_resend:
                return
        print_time = max(print_time, self.last_print_time + PIN_MIN_TIME)
        if self.is_pwm:
            self.mcu_pin.set_pwm(print_time, value, cycle_time)
        else:
            self.mcu_pin.set_digital(print_time, value)
        self.last_value = value
        self.last_cycle_time = cycle_time
        self.last_print_time = print_time
        if self.resend_interval and self.resend_timer is None:
            self.resend_timer = self.reactor.register_timer(
                self._resend_current_val, self.reactor.NOW)
    cmd_SET_PIN_help = "Set the value of an output pin"
    def cmd_SET_PIN(self, gcmd):
        value = gcmd.get_float('VALUE', minval=0., maxval=self.scale)
        value /= self.scale
        cycle_time = gcmd.get_float('CYCLE_TIME', self.default_cycle_time,
                                    above=0., maxval=MAX_SCHEDULE_TIME)
        if not self.is_pwm and value not in [0., 1.]:
            raise gcmd.error("Invalid pin value")
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback(
            lambda print_time: self._set_pin(print_time, value, cycle_time))
        
    def set_pin(self, value, cycle_time = None):
        if cycle_time is None:
            cycle_time = self.default_cycle_time
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback(
            lambda print_time: self._set_pin(print_time, value, cycle_time))

    def _resend_current_val(self, eventtime):
        if self.last_value == self.shutdown_value:
            self.reactor.unregister_timer(self.resend_timer)
            self.resend_timer = None
            return self.reactor.NEVER

        systime = self.reactor.monotonic()
        print_time = self.mcu_pin.get_mcu().estimated_print_time(systime)
        time_diff = (self.last_print_time + self.resend_interval) - print_time
        if time_diff > 0.:
            # Reschedule for resend time
            return systime + time_diff
        self._set_pin(print_time + PIN_MIN_TIME,
                      self.last_value, self.last_cycle_time, True)
        return systime + self.resend_interval


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
        
        self.hes_gauge_pins = []
        pin_names = config.get('hes_gauge_pins',None)
        if pin_names is not None:
            i = 0
            for pin_name in pin_names.split(","):
                self.hes_gauge_pins.append(RTOutputPin(config, pin_name, str(i)))
                i += 1

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
        #logging.info("read value: %s", read_value)
        n_pins = len(self.hes_gauge_pins)
        for i in range(0, n_pins):
            if read_value > (i+1)/n_pins:
                value = 1.0
            elif read_value > i/n_pins and read_value <= (i+1)/n_pins:
                value = (read_value - i/n_pins) / (1/n_pins)
            else:
                value = 0.0
            #logging.info("%i:%f" % (i,value))
            
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.register_lookahead_callback(
                lambda print_time: self.hes_gauge_pins[i]._set_pin(print_time, value,
                            self.hes_gauge_pins[i].cycle_time))
        
        
        control = self._pid.pressure_update(read_time, read_value, self._set_point)
        if self.enable:
            rotation_distance= self.base_rotation_distance - control * MAX_ROTATION_VARIANCE
            self.base_stepper.set_rotation_distance(rotation_distance)
            #logging.info("Rotation Distance: %s", rotation_distance)

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
