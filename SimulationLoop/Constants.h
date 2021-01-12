#pragma once

#define WND_WIDTH  800
#define WND_HEIGHT 800

// CONSTRAINT_BOARD not defined means that will use STATIC bodies instead of constraints
//#define CONSTRAINT_BOARD

// consider angular velocity when defined
//#define ANGULAR_VELOCITY

// colors
#define BLACK math::Vector3D(0.f, 0.f, 0.f)
#define WHITE math::Vector3D(1.f, 1.f, 1.f)
#define GRAY  math::Vector3D(0.5f, 0.5f, 0.5f)
#define GREEN math::Vector3D(0.f, 1.0f, 0.0f)

// Physics
#define TIME_SCALE_RATE 0.1f
#define FRICTION_RATE 0.1f
#define BALL_SIZE_RATE 0.1f
#define RESTORATION_RATE 0.1f
#define GRAVITY -9.81f
//#define DAMPING 0.98f
#define DAMPING 1.f

// Define integration
#define EULER_INTEGRATION
#define ACCURATE_EULER_INTEGRATION
//#define VERLET_INTEGRATION

// Changed using keys
#define FRICTION_MAG_DEFAULT 0.1f
#define RESTITUTION_MAG_DEFAULT 0.1f
#define BALL_SIZE_DEFAULT 0.5f

// Board's elements creation
#define BALL_TOTAL 50
#define BALL_MASS  1.f
#define PEG_SPHERE_SIZE = 3.f
#define PEG_COLOR WHITE

// Keyboard 
#define VK_W 0x57
#define VK_S 0x53
#define VK_D 0x44
#define VK_A 0x41
#define VK_R 0x52
#define VK_N 0x4E
#define VK_P 0x50
#define VK_U 0x55
#define VK_J 0x4A
#define VK_I 0x49
#define VK_K 0x4B
#define VK_T 0x54
#define VK_B 0x42
#define VK_O 0x4F
#define VK_L 0x4C

#define VK_0 0x30
#define VK_1 0x31
#define VK_2 0x32
#define VK_3 0x33
#define VK_4 0x34
#define VK_5 0x35
#define VK_6 0x36
#define VK_7 0x37
#define VK_8 0x38
#define VK_9 0x39