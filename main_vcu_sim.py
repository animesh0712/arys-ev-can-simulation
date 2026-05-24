import can
import struct
import time
import threading
import math

# ==========================================
# CAN Bus Configuration
# ==========================================
# Thread-safe interface prevents GIL collision during asynchronous writes
can_interface = 'virtual'
channel = 'can0'

bus = can.ThreadSafeBus(interface=can_interface, channel=channel, bitrate=500000, receive_own_messages=True)
logger = can.Logger('can_logs.csv')
notifier = can.Notifier(bus, [logger])

# ==========================================
# Shared ECU Base Class
# ==========================================
class ECUNode(threading.Thread):
    def __init__(self, name, loop_rate_hz):
        super().__init__()
        self.name = name
        self.loop_delay = 1.0 / loop_rate_hz
        self.running = threading.Event()
        self.running.set()
        
    def run(self):
        while self.running.is_set():
            start_time = time.time()
            self.update_physics()
            self.dispatch_can()
            self.listen_can()
            
            # Maintain strict cycle time matching RTOS behavior
            elapsed = time.time() - start_time
            sleep_time = self.loop_delay - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    def stop(self):
        self.running.clear()
        
    def update_physics(self):
        pass # Overridden by child classes
        
    def dispatch_can(self):
        pass # Overridden by child classes

    def listen_can(self):
        pass # Overridden by child classes

# ==========================================
# Battery Management System (BMS) Node
# ==========================================
class BMSNode(ECUNode):
    def __init__(self):
        super().__init__("BMS", loop_rate_hz=20) # 50ms cycle
        # 10s10p Pack parameters
        self.voltage = 37.0 
        self.soc = 100.0
        self.temp = 25.0
        self.current = 0.0
        self.r_int = 0.05 # Normal internal resistance Ohms
        self.capacity_Ah = 50.0
        self.contactor = 1 # Closed
        self.fault = 0
        
        # dynamic limits
        self.max_dsg = 200.0
        self.max_chg = 50.0
        self.max_torque = 150.0

        # Fault Injection Flag
        self.inject_thermal_runaway = False

    def update_physics(self):
        # Fault injection logic
        if self.inject_thermal_runaway:
            self.r_int = 0.8 # Massive resistance spike
            
        # Physics: Joule heating
        joule_heat = (self.current ** 2) * self.r_int
        cooling = 5.0 * (self.temp - 25.0) # Convective cooling
        temp_derivative = (joule_heat - cooling) / 1000.0
        self.temp += temp_derivative * self.loop_delay
        
        # Physics: Coulomb Counting
        soc_drain = (self.current * self.loop_delay) / (self.capacity_Ah * 3600.0)
        self.soc = max(0.0, self.soc - (soc_drain * 100))
        
        # Voltage Sag
        self.voltage = (3.7 * 10) - (self.current * self.r_int)
        
        # Thermal Derating Logic
        if self.temp > 55.0:
            self.max_torque = 40.0 # Derate torque
            self.fault |= 0x04     # Set over-temp fault bit
        else:
            self.max_torque = 150.0
            
    def listen_can(self):
        # Non-blocking read to get current load from MCU
        msg = bus.recv(timeout=0.001)
        if msg and msg.arbitration_id == 0x130:
            actual_torque = struct.unpack('<h', msg.data[0:2])[ 0 ] * 0.01
            rpm = struct.unpack('<h', msg.data[2:4])[ 0 ]
            power_watts = (actual_torque * rpm * 2 * math.pi) / 60
            self.current = power_watts / self.voltage if self.voltage > 0 else 0
            
    def dispatch_can(self):
        # Pack BMS_STATUS (0x110)
        v_raw = int(self.voltage / 0.1)
        i_raw = int(self.current / 0.1)
        soc_raw = int(self.soc / 0.5)
        t_raw = int(self.temp + 40.0)
        
        data_110 = struct.pack('<H h B B B B', v_raw, i_raw, soc_raw, t_raw, self.contactor, self.fault)
        bus.send(can.Message(arbitration_id=0x110, data=data_110, is_extended_id=False))
        
        # Pack BMS_LIMITS (0x111)
        md_raw = int(self.max_dsg / 0.1)
        mc_raw = int(self.max_chg / 0.1)
        mt_raw = int(self.max_torque / 0.01)
        
        data_111 = struct.pack('<H H H x x', md_raw, mc_raw, mt_raw)
        bus.send(can.Message(arbitration_id=0x111, data=data_111, is_extended_id=False))

# ==========================================
# Vehicle Control Unit (VCU) Node
# ==========================================
class VCUNode(ECUNode):
    def __init__(self):
        super().__init__("VCU", loop_rate_hz=50) # 20ms cycle
        self.throttle_input = 0.0 # 0-100%
        self.torque_cmd = 0.0
        self.inverter_en = 1
        self.drive_mode = 1 # Fwd
        
        self.bms_torque_limit = 150.0
        self.last_bms_msg_time = time.time()
        
    def set_throttle(self, val):
        self.throttle_input = val

    def listen_can(self):
        msg = bus.recv(timeout=0.001)
        if msg:
            if msg.arbitration_id == 0x110:
                self.last_bms_msg_time = time.time()
            elif msg.arbitration_id == 0x111:
                self.bms_torque_limit = struct.unpack('<H', msg.data[4:6])[ 0 ] * 0.01
                
    def update_physics(self):
        # Watchdog: Loss of Communication
        if (time.time() - self.last_bms_msg_time) > 0.200:
            self.inverter_en = 0 # Limp mode / safe state
            self.torque_cmd = 0.0
            return
            
        # Drive Permission & Throttle Mapping
        raw_cmd = (self.throttle_input / 100.0) * 150.0
        
        # Clipping against BMS limits
        self.torque_cmd = min(raw_cmd, self.bms_torque_limit)

    def dispatch_can(self):
        # Pack VCU_COMMAND (0x120)
        t_raw = int(self.torque_cmd / 0.01)
        data_120 = struct.pack('<h B B x x x x', t_raw, self.inverter_en, self.drive_mode)
        bus.send(can.Message(arbitration_id=0x120, data=data_120, is_extended_id=False))

# ==========================================
# Motor Control Unit (MCU) Node
# ==========================================
class MCUNode(ECUNode):
    def __init__(self):
        super().__init__("MCU", loop_rate_hz=50) # 20ms cycle
        self.actual_torque = 0.0
        self.rpm = 0.0
        self.cmd_torque = 0.0
        self.inv_en = 0
        
        # Kinematics parameters
        self.inertia = 2.5 # kg*m^2
        self.aero_drag_coeff = 0.005
        self.rolling_counter = 0

    def listen_can(self):
        msg = bus.recv(timeout=0.001)
        if msg and msg.arbitration_id == 0x120:
            self.cmd_torque = struct.unpack('<h', msg.data[0:2])[ 0 ] * 0.01
            self.inv_en = msg.data[1]
            
    def update_physics(self):
        if self.inv_en:
            # Low pass filter for actual torque (FOC response time)
            self.actual_torque += 0.2 * (self.cmd_torque - self.actual_torque)
        else:
            self.actual_torque = 0.0
            
        # Kinematics
        drag = (self.rpm * 0.1) + (self.aero_drag_coeff * (self.rpm ** 2) / 1000.0)
        net_torque = self.actual_torque - drag
        
        alpha = net_torque / self.inertia
        self.rpm += alpha * self.loop_delay
        if self.rpm < 0: self.rpm = 0.0

    def dispatch_can(self):
        t_raw = int(self.actual_torque / 0.01)
        rpm_raw = int(self.rpm)
        inv_temp_raw = int(45.0 + 40.0)
        mot_temp_raw = int(50.0 + 40.0)
        self.rolling_counter = (self.rolling_counter + 1) % 256
        
        data_130 = struct.pack('<h h B B B B', t_raw, rpm_raw, inv_temp_raw, mot_temp_raw, 0, self.rolling_counter)
        bus.send(can.Message(arbitration_id=0x130, data=data_130, is_extended_id=False))

# ==========================================
# Main Execution Block
# ==========================================
if __name__ == "__main__":
    bms = BMSNode()
    vcu = VCUNode()
    mcu = MCUNode()
    
    bms.start()
    vcu.start()
    mcu.start()
    
    try:
        print("Simulation Running. Press Ctrl+C to exit.")
        time.sleep(1)
        
        print("=> Scenario 1: Normal Acceleration (Throttle 80%)")
        vcu.set_throttle(80.0)
        time.sleep(3)
        
        print("=> Scenario 2: Fault Injection (Thermal Runaway)")
        bms.inject_thermal_runaway = True
        time.sleep(4)
        
        print("=> Scenario 3: Comm Timeout (BMS Disconnected)")
        bms.stop() # Simulate BMS hard crash
        time.sleep(2)
        
    except KeyboardInterrupt:
        pass
    finally:
        bms.stop()
        vcu.stop()
        mcu.stop()
        notifier.stop()
        logger.stop()
        bus.shutdown()
        print("Simulation Shutdown Successfully.")