Electric Vehicle Powertrain CAN Simulation
Project Scope
This repository contains a software-only Controller Area Network (CAN) simulation for an Electric Vehicle (EV) powertrain. The project models the real-time communication and physical state dynamics between three core vehicle computers:

Battery Management System (BMS): Simulates a 10s10p 21700 NMC battery pack. It calculates real-time State of Charge (SoC) via Coulomb counting, internal resistance voltage sag, and thermal heat generation. It dynamically broadcasts power limits to the vehicle network.

Vehicle Control Unit (VCU): Acts as the master supervisory controller. It translates driver throttle input into torque commands, while strictly enforcing drive permission logic based on the thermal and electrical limits dictated by the BMS.

Motor Control Unit (MCU): Receives torque commands and simulates the mechanical rotational kinematics (RPM and actual output torque) of the traction motor based on system inertia and aerodynamic drag.

Scenarios Validated:

Normal Acceleration (Steady-State Operation)
Fault Injection: Thermal Runaway & Dynamic Derating
Loss of Communication: Hard ECU Crash & Safe State Execution

Technologies Used:
Python 3
python-can library
Virtual CAN bus (virtual interface)
python-can library

Virtual CAN bus (virtual interface)
