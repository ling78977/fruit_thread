#include "../src/module/arm/arm.h"

int main() {
  arm::Arm_Device arm1;
  arm1.Arm_serial_servo_write6(0, 0, 0, 0, 0, 0, 1500);

  return 0;
}