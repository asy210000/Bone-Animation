# Bone Animation

## Description
This project implements a bone animation system using transformations and Jacobian Transpose IK (Inverse Kinematics). The system defines a hierarchical structure of bones with scaling, rotation, and translation matrices to control movement and positioning.

## Features
- Hierarchical bone structure
- Rotation and translation calculations using GLM (OpenGL Mathematics)
- End-effector movement using Jacobian Transpose method for inverse kinematics
- Configurable root position, bone scaling, and rotation degrees
- Ability to reset the bone system to its default state

## Dependencies
- [GLM (OpenGL Mathematics)](https://github.com/g-truc/glm)

## Installation
1. Clone this repository:
   ```sh
   git clone https://github.com/yourusername/bone_animation.git
   ```
2. Include the required GLM library in your project.
3. Compile the source files with your C++ compiler.

## Usage
### Initialization
- Modify the `root_position`, `target`, and `scale_vector` as needed in the `init()` function of `Bone_Animation.cpp`.

### Running the Animation
- Call `update(float delta_time)` in the game loop to update the bone positions.
- Use `reset()` to return bones to their default state.

### Adjusting Video Path (If Necessary)
- If you are using visualization, ensure any video paths or graphical output methods are correctly set up according to your environment.

## File Structure
```
Bone_Animation.h - Header file defining the bone animation class.
camera.h - Manages the camera perspective and movement.
lighting.h - Handles scene lighting calculations.
main.cpp - Entry point of the application.
object.h - Defines objects within the scene.
renderer.cpp - Handles rendering logic for the animation.
renderer.h - Header file for rendering functions.
shader.h - Manages shader programs for rendering.
```

## License
This project is licensed under the MIT License.

