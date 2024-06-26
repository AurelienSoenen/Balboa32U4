# Balboa 32U4

## BalancerLinearController Folder

This folder contains three files essential for stabilizing the Balboa 32U4 robot: `BalancerLinearController.ino`, `Balance.cpp`, and `Balance.h`.

### Functionality

This code is designed to stabilize the Balboa 32U4 in various configurations:

1. **Initial Positioning**: If the Balboa is positioned at an angle of less than 5 degrees to the vertical.
2. **Button B Press**: When button B is pressed while the robot is lying down, the Balboa initializes its sensors, moves into a vertical position, and stabilizes itself.

### Configuration Changes

You can change the configuration by pressing buttons A and C:
- **Button A**: Increases the configuration.
- **Button C**: Decreases the configuration.

#### Configurations

1. **First Configuration**: 
   - The Balboa stabilizes with a medium-response controller.
   - It follows a specific reference trajectory, moving the robot from position A to position B and back to position A.

2. **Second Configuration**: 
   - The Balboa stabilizes with a fast-response controller.
   - After 5 seconds, Gaussian perturbations are added to perform data-driven control searches.

3. **Third Configuration**: 
   - The Balboa stabilizes with a slow-response controller. 

### Usage

To use this code, upload the `BalancerLinearController.ino` file to your Balboa 32U4. Ensure that `Balance.cpp` and `Balance.h` are in the same directory. Use the buttons on the Balboa to select and switch between configurations as described above.
