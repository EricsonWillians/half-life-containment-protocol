# Half-Life: Containment Protocol

**A survival horror/action horror conversion mod for Half-Life (GoldSource & Xash3D)**

[![Windows Build Status](https://img.shields.io/badge/build-passing-brightgreen)](link)
[![License](https://img.shields.io/badge/license-Custom-blue)](link)
[![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20Android-lightgrey)](link)

---

## 🎯 Project Vision

**Half-Life: Containment Protocol** transforms the classic Half-Life experience into a spine-chilling survival horror adventure. Built on the robust foundation of HLSDK-Portable, this mod introduces a revolutionary **third-person precision aiming system** that delivers Serious Sam-level shooting accuracy while maintaining the cinematic tension of games like Resident Evil 4–6.

### 🎮 Core Philosophy
- **Survival Horror Focus**: Atmospheric tension, resource management, and psychological horror
- **Action Horror Elements**: Intense combat encounters with precise, skill-based mechanics  
- **Cinematic Experience**: Third-person perspective enhances immersion and environmental storytelling
- **Technical Excellence**: Modern engine enhancements while preserving GoldSource authenticity

---

## 🌟 Key Features

### 👁️ **Enhanced Third-Person Camera System**
Our flagship feature completely reimagines Half-Life's perspective:

- **Precision Aiming**: Camera-center raycast system ensures crosshair accuracy
- **Dynamic Shoulder Camera**: Smooth transitions between left/right shoulder positions
- **Environmental Awareness**: Automatic shoulder swapping when near walls/obstacles
- **Collision Detection**: Intelligent camera positioning prevents clipping through geometry
- **Cinematic Framing**: Maintains visual appeal while preserving gameplay functionality

### 🎯 **Advanced Aiming Mechanics**
- **Dual-Ray System**: Separate camera and muzzle calculations for authentic ballistics
- **Recoil Simulation**: Realistic weapon kick with pattern-based accumulation
- **Aim Assist**: Optional magnetism for controller users (multiplayer-aware)
- **Weapon Alignment**: Visual consistency between crosshair and weapon orientation
- **Obstruction Detection**: Real-time validation of shot paths

### 🔧 **Technical Enhancements**
- **Cross-Platform Support**: Windows, Linux, Android, Nintendo Switch, PlayStation Vita
- **Modern Build System**: CMake integration with multiple compiler support
- **Performance Optimized**: Maintains 60+ FPS on target platforms
- **Debug Visualization**: Developer tools for fine-tuning aiming mechanics

---

## 🎨 Horror Transformation

### 📱 **User Interface Overhaul**
- **Minimalist HUD**: Reduced UI elements for increased immersion
- **Dynamic Crosshair**: Contextual feedback for aiming precision
- **Resource Indicators**: Subtle ammunition and health displays
- **Atmospheric Elements**: Environmental interaction prompts

### 🌙 **Atmospheric Systems**
- **Enhanced Lighting**: Dramatic shadows and dynamic illumination
- **Audio Design**: 3D positional audio with environmental reverb
- **Particle Effects**: Improved smoke, sparks, and atmospheric details
- **Weather Systems**: Rain, fog, and environmental hazards

### 👹 **Gameplay Mechanics**
- **Resource Scarcity**: Limited ammunition encourages strategic thinking
- **Stealth Elements**: Third-person perspective enables tactical positioning
- **Environmental Puzzles**: Enhanced spatial awareness for complex interactions
- **Survival Elements**: Health regeneration limitations and stamina systems

---

## 🚀 Getting Started

### 📋 **Prerequisites**
- Half-Life (Steam/WON version) or Xash3D engine
- Visual Studio 2019+ (Windows) or GCC/Clang (Linux)
- CMake 3.16 or newer
- Git with submodule support

### 🛠️ **Quick Build**

#### Windows
```bash
git clone --recursive https://github.com/YourRepo/hlsdk-containment-protocol
cd hlsdk-containment-protocol
cmake -A Win32 -B build -DCONTAINMENT_PROTOCOL=ON
cmake --build build --config Release
```

#### Linux
```bash
git clone --recursive https://github.com/YourRepo/hlsdk-containment-protocol
cd hlsdk-containment-protocol
cmake -DCMAKE_BUILD_TYPE=Release -B build -S . -DCONTAINMENT_PROTOCOL=ON
cmake --build build
```

### ⚙️ **Configuration Options**
```bash
# Core mod features
-DCONTAINMENT_PROTOCOL=ON          # Enable mod-specific features
-DTHIRD_PERSON_PRECISION=ON        # Enhanced aiming system
-DSURVIVAL_HORROR_MODE=ON          # Atmospheric enhancements
-DDEBUG_AIMING_TRACES=OFF          # Development visualization

# Third-person aiming system
-DTPS_SHOULDER_OFFSET=16           # Camera shoulder distance
-DTPS_AIM_ASSIST=0.15             # Controller aim magnetism
-DTPS_PRECISION_MODE=ON            # High-accuracy mode
```

---

## 🎛️ **Third-Person Aiming System**

### 🎯 **Core Mechanics**

Our revolutionary aiming system achieves pixel-perfect accuracy through advanced raycast technology:

1. **Camera-Center Targeting**: Primary raycast from screen center determines target point
2. **Muzzle Validation**: Secondary raycast ensures clear firing path
3. **Dynamic Crosshair**: Real-time feedback reflects actual aim point
4. **Weapon Alignment**: Visual weapon orientation matches targeting direction

### 🎮 **Console Commands**
```bash
# Camera configuration
tps_shoulder_offset "16"           # Shoulder camera distance
tps_camera_distance "80"          # Camera-to-player distance
tps_collision_detection "1"       # Enable camera collision

# Aiming precision
tps_precision_mode "1"             # Enable high-accuracy mode
tps_crosshair_smoothing "8.5"     # Crosshair movement smoothing
tps_weapon_alignment "1"          # Visual weapon alignment

# Dynamic features
tps_shoulder_swap "1"              # Auto shoulder switching
tps_aim_assist "0.15"             # Aim magnetism strength
tps_debug_traces "0"               # Developer visualization
```

### 🔧 **Integration Guide**

#### For Weapon Developers
```cpp
// Get crosshair target position
vec3_t target_point;
V_GetCrosshairTarget(target_point);

// Get weapon muzzle position
vec3_t muzzle_pos;
V_GetMuzzlePosition(muzzle_pos);

// Check for obstructions
if (!V_IsAimObstructed()) {
    // Fire projectile from muzzle to target
    FireProjectile(muzzle_pos, target_point);
}

// Apply accuracy modifier
float accuracy = V_GetWeaponAccuracyModifier();
```

---

## 🗂️ **Project Structure**

```
hlsdk-containment-protocol/
├── 📁 cl_dll/              # Client-side code
│   ├── 📄 view.cpp         # Enhanced camera system
│   ├── 📄 hud.cpp          # UI modifications
│   └── 📄 weapons/         # Client weapon code
├── 📁 dlls/                # Server-side code
│   ├── 📄 player.cpp       # Player mechanics
│   ├── 📄 weapons/         # Weapon implementations
│   └── 📄 monsters/        # AI enhancements
├── 📁 game_shared/         # Shared game logic
├── 📁 common/              # Common utilities
├── 📁 engine/              # Engine interface
├── 📁 pm_shared/           # Physics code
└── 📁 utils/               # Development tools
```

---

## 🤝 **Contributing**

We welcome contributions from the Half-Life modding community!

### 🔀 **Development Workflow**
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-addition`)
3. Test thoroughly on multiple platforms
4. Submit a pull request with detailed description

### 📝 **Coding Standards**
- Follow existing code style and conventions
- Include comprehensive comments for complex systems
- Test third-person aiming accuracy across different scenarios
- Maintain compatibility with both GoldSource and Xash3D

### 🐛 **Bug Reports**
Please include:
- Platform and engine version
- Steps to reproduce
- Expected vs actual behavior
- Console output and error messages

---

## 📄 **License & Credits**

**Half-Life: Containment Protocol** is based on HLSDK-Portable by FWGS team.

- Original Half-Life SDK: © Valve Corporation
- HLSDK-Portable: © FWGS team
- Containment Protocol modifications: © Project contributors

### 🙏 **Special Thanks**
- **Valve Software** - For the legendary Half-Life engine
- **FWGS Team** - For the excellent portable SDK
- **Survival Horror Community** - For inspiration and feedback
- **Beta Testers** - For helping perfect the aiming system

---

## 📞 **Contact & Support**

- **Issues**: [GitHub Issues](link)
- **Discussions**: [GitHub Discussions](link)
- **Discord**: [Community Server](link)
- **Wiki**: [Documentation](link)

---

<div align="center">

**Experience Half-Life like never before.**

*Precision aiming meets survival horror in the ultimate GoldSource conversion.*

[![Download Latest](https://img.shields.io/badge/Download-Latest%20Release-brightgreen)](link)
[![Documentation](https://img.shields.io/badge/Read-Documentation-blue)](link)
[![Community](https://img.shields.io/badge/Join-Community-purple)](link)

</div>