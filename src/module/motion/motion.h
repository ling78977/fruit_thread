#ifndef FRUIT_SRC_MODULE_MOTION_MOTION_H_
#define FRUIT_SRC_MODULE_MOTION_MOTION_H_
#include "src/module/arm/arm.h"

namespace fruit {
namespace motion {

fruit::arm::ArmDevice &arm_device = fruit::arm::ArmDevice::getInstance();
void armStandby(int direction, int const claw = 50);
void armLook(int direction);
void armScout(int direction);
void armGrab(std::vector<double> data_ik);
void armMidway();
void putFruitToCup(int cup);
void putFruitsToBasket(int direction, int cup);
}
}  // namespace fruit

#endif