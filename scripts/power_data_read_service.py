#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.srv import PowerDataRead

class PowerDataReadService(Node):
    def __init__(self):
        super().__init__('power_data_read_service')
        self.srv = self.create_service(PowerDataRead, 'Power_data_read_cmd', self.callback)

    def callback(self, request, response):
        response.d0_battery_voltage = 540
        response.d1_system_current = 15
        response.d2_charge_current = 0
        response.d3_charge_signal = 14
        response.d4_inport_status = 0
        response.d5_outport_status = 0
        response.d6_power_status = 63
        response.d7_charger_terminal_status = 1
        response.d8_temperature0 = 326
        response.d9_temperature1 = 271
        response.d10_mobd_inport_status = 15
        response.d11_mobd_outport_status = 0
        response.command_result = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PowerDataReadService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
