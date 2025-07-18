import can
import time
import math
import struct
import csv
import os
import json

try:
    with open('config.json', 'r') as f:
        config = json.load(f)
except FileNotFoundError:
    print("Błąd: Nie znaleziono pliku 'config.json'.")
    exit()

COMMANDS = {key: int(value, 16) for key, value in config['commands'].items()}
GLOBALS = config['global_params']
CAN_CONFIG = config['can']
LOGGING_CONFIG = config['logging']


class KalmanFilter1D:
    def __init__(self, R=0.01, Q_angle=0.001, Q_velocity=0.003):
        self.R, self.Q_angle, self.Q_velocity = R, Q_angle, Q_velocity
        self.angle, self.bias = 0.0, 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]
    
    def update(self, new_angle, dt):
        rate = new_angle - self.angle
        self.angle += dt * (rate - self.bias)
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_velocity * dt
        S = self.P[0][0] + self.R
        K = [self.P[0][0] / S, self.P[1][0] / S]
        y = new_angle - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y
        P00_temp, P01_temp = self.P[0][0], self.P[0][1]
        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp
        return self.angle, self.bias

class MotorController:
    def __init__(self, can_bus, motor_config):
        self.bus = can_bus
        self.name = motor_config['name']
        self.id = motor_config['id']
        self.reduction_ratio = motor_config['reduction_ratio']
        self.angle_offset = motor_config['angle_offset_degrees']
        self.kt_nm_per_a = motor_config['kt_nm_per_a']
        self.max_torque = motor_config['max_torque_nm']
        self.torque_limit = motor_config['torque_limit_nm']
        control_params = motor_config['control']
        self.mass = control_params['mass_kg']
        self.com_dist = control_params['com_distance_m']
        self.damping = control_params['damping_reg']
        self.smoothing_factor = control_params['smoothing_factor']
        friction_params = motor_config['friction']
        self.coulomb_fric = friction_params['coulomb_iq']
        self.viscous_fric = friction_params['viscous_coeff_iq']
        self.kf = KalmanFilter1D(R=0.2)
        self.is_on = False
        self.is_holding = False
        self.current_torque_nm = 0.0
        self.motor_angle_degrees = 0.0

    def motor_power(self, state):
        cmd = COMMANDS['motor_on'] if state else COMMANDS['motor_off']
        message = can.Message(arbitration_id=COMMANDS['tx_rx_base_id'] + self.id, data=[cmd, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False)
        self.bus.send(message)
        self.is_on = state
        time.sleep(0.01)

    def set_torque_and_get_angle(self, torque_nm):
        torque_nm = max(-self.torque_limit, min(self.torque_limit, torque_nm))
        iq_control_raw = int((torque_nm / self.max_torque) * GLOBALS['iq_control_range'])
        iq_control = max(-GLOBALS['iq_control_range'], min(GLOBALS['iq_control_range'], iq_control_raw))
        iq_bytes = struct.pack('<h', iq_control)
        data_payload = [COMMANDS['torque_closed_loop'], 0, 0, 0, iq_bytes[0], iq_bytes[1], 0, 0]
        message = can.Message(arbitration_id=COMMANDS['tx_rx_base_id'] + self.id, data=data_payload, is_extended_id=False)
        
        try:
            self.bus.send(message)
            response = self.bus.recv(timeout=0.01) 
            if response and response.arbitration_id == (COMMANDS['tx_rx_base_id'] + self.id):
                raw_encoder = struct.unpack('<H', response.data[6:8])[0]
                self.motor_angle_degrees = (raw_encoder / GLOBALS['encoder_max_value']) * (360.0 * self.reduction_ratio)
                return True
        except can.CanError as e:
            print(f"Błąd komunikacji CAN dla silnika ID {self.id}: {e}")
        return False
        
    def _convert_iq_to_nm(self, iq_value):
        current_A = (iq_value / GLOBALS['iq_control_range']) * GLOBALS['max_current_a_for_iq_range']
        return current_A * self.kt_nm_per_a

    def _calculate_friction_torque(self, angular_velocity_dps):
        coulomb_comp_iq = self.coulomb_fric * math.tanh(angular_velocity_dps * 5.0)
        viscous_comp_iq = self.viscous_fric * angular_velocity_dps
        return self._convert_iq_to_nm(coulomb_comp_iq + viscous_comp_iq)

    def update(self, dt):
        calibrated_motor_angle = self.motor_angle_degrees - self.angle_offset
        output_angle_degrees = calibrated_motor_angle / self.reduction_ratio
        output_angle_radians = math.radians(output_angle_degrees)
        
        _, angular_velocity_rad_s = self.kf.update(output_angle_radians, dt)
        angular_velocity_dps = math.degrees(angular_velocity_rad_s)
        
        is_in_hold_zone = (GLOBALS['hold_zone_min_output_angle'] < output_angle_degrees < GLOBALS['hold_zone_max_output_angle'])

        target_torque_nm = 0.0
        if self.is_holding:
            if not is_in_hold_zone:
                self.is_holding = False
        else:
            if is_in_hold_zone and abs(angular_velocity_rad_s) < GLOBALS['settle_velocity_threshold_rps']:
                self.is_holding = True
            else:
                gravity_torque = (self.mass * GLOBALS['gravity'] * self.com_dist * math.sin(output_angle_radians))
                friction_torque = self._calculate_friction_torque(angular_velocity_dps)
                damping_torque = -self.damping * angular_velocity_rad_s
                target_torque_nm = gravity_torque + friction_torque + damping_torque
                
        if not self.is_holding:
            self.current_torque_nm += self.smoothing_factor * (target_torque_nm - self.current_torque_nm)
        else:
            self.current_torque_nm = 0.0

        self.set_torque_and_get_angle(self.current_torque_nm)

        print(f"[{self.name}] Kąt: {output_angle_degrees:6.2f}° | V: {angular_velocity_dps:7.2f} dps | T: {self.current_torque_nm:6.3f} Nm | Hold: {self.is_holding}   ", end='\n')


def main():
    print("Inicjalizacja magistrali CAN...")
    try:
        bus = can.interface.Bus(bustype='slcan', channel=CAN_CONFIG['interface'], bitrate=CAN_CONFIG['bitrate'])
    except can.CanError as e:
        print(f"Błąd inicjalizacji magistrali CAN: {e}")
        return

    controllers = [MotorController(bus, motor_conf) for motor_conf in config['motors']]
    print(f"Utworzono {len(controllers)} kontrolerów silników.")

    try:
        for controller in controllers:
            controller.motor_power(True)
            print(f"Silnik '{controller.name}' (ID: {controller.id}) włączony.")
        
        print("\nUruchomiono pętlę sterującą...")
        last_loop_time = time.time()
        
        while True:
            current_time = time.time()
            dt = current_time - last_loop_time
            if dt < 0.001: continue
            last_loop_time = current_time

            for controller in controllers:
                controller.update(dt)
            
            time.sleep(0.005) 

    except KeyboardInterrupt:
        print("\nZatrzymano pętlę sterującą przez użytkownika.")
    finally:
        print("\nWyłączanie silników...")
        if 'bus' in locals() and bus is not None:
            for controller in controllers:
                if controller.is_on:
                    controller.set_torque_and_get_angle(0)
                    controller.motor_power(False)
                    print(f"Silnik '{controller.name}' (ID: {controller.id}) wyłączony.")
            bus.shutdown()
        print("Program zakończony.")

if __name__ == "__main__":
    main()