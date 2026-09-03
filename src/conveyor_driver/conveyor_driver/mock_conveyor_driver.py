#!/usr/bin/env python3

import rclpy

from conveyor_driver.conveyor_node import ConveyorLifecycleNode


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorLifecycleNode(driver_type="mock")
    try:
        node.trigger_configure()
        node.trigger_activate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.trigger_deactivate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
