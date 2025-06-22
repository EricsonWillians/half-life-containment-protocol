//========= Copyright (c) 1996-2002, Valve LLC, All rights reserved. ============
//
// Purpose: Enhanced Survival Horror Camera System Header
//
// $NoKeywords: $
//=============================================================================

#pragma once
#if !defined(_CAMERA_H_)
#define _CAMERA_H_

// Forward declare vec4_t if not available, usually in com_model.h
// This prevents errors if this header is included before the main engine headers.
#ifndef vec4_t
typedef float vec4_t[4];
#endif

// Camera zone structure for fixed angle cameras
typedef struct camera_zone_s
{
    vec3_t mins, maxs;          // Zone boundaries
    vec3_t cam_position;        // Fixed camera position
    vec3_t cam_angles;          // Fixed camera angles
    float transition_time;      // Time to transition to this camera
    int zone_id;               // Unique zone identifier
    bool is_active;            // Is this zone currently active
    float fov;                 // Field of view for this camera
    vec3_t focus_point;        // Point the camera should focus on
    bool follow_player;        // Should camera follow player movement
    float follow_speed;        // Speed of camera following
    float shake_intensity;     // Atmospheric shake intensity
    float ambient_sway;        // Subtle ambient camera movement
} camera_zone_t;

// Camera transition structure
typedef struct camera_transition_s
{
    bool in_transition;
    float transition_progress;
    float transition_duration;
    vec3_t start_pos, end_pos;
    vec3_t start_angles, end_angles;
    float start_fov, end_fov;
    int transition_type;       // Linear, smooth, cinematic
} camera_transition_t;

// Cinematic camera keyframe
typedef struct camera_keyframe_s
{
    float time;
    vec3_t position;
    vec3_t angles;
    float fov;
    vec3_t target_pos;         // For look-at cameras
    float ease_in, ease_out;   // Easing parameters
} camera_keyframe_t;

// Cinematic sequence
typedef struct camera_sequence_s
{
    camera_keyframe_t *keyframes;
    int num_keyframes;
    float total_duration;
    bool is_playing;
    float current_time;
    bool loop;
    int current_keyframe;
} camera_sequence_t;

// Enhanced camera state
typedef struct camera_state_s
{
    // Current camera mode
    int mode;                  // 0=first person, 1=third person, 2=fixed angle, 3=cinematic
    
    // Fixed angle camera system
    camera_zone_t *zones;
    int num_zones;
    int current_zone;
    int previous_zone;
    
    // Camera transitions
    camera_transition_t transition;
    
    // Cinematic system
    camera_sequence_t *sequences;
    int num_sequences;
    int active_sequence;
    
    // Atmospheric effects
    float atmospheric_shake;
    float tension_level;       // 0.0 to 1.0, affects camera behavior
    vec3_t shake_offset;
    vec3_t sway_offset;
    
    // Player tracking
    vec3_t player_velocity;
    vec3_t predicted_position;
    float look_ahead_distance;
    
    // Collision detection
    bool collision_enabled;
    float collision_radius;
    
    // Visual effects
    float vignette_intensity;
    float color_correction[3]; // RGB multipliers
    float contrast_boost;
    
    // Audio-reactive camera
    float audio_intensity;
    float audio_shake_scale;

    // --- MEMBERS FOR CROSSHAIR/AIMING ---
    vec3_t crosshair_world_pos;
    vec3_t crosshair_screen_pos;
    vec3_t aim_target;
    bool   crosshair_obstructed;
    float  crosshair_visibility;
    
} camera_state_t;

// Camera modes
#define CAM_MODE_FIRSTPERSON    0
#define CAM_MODE_THIRDPERSON    1
#define CAM_MODE_FIXEDANGLE     2
#define CAM_MODE_CINEMATIC      3
#define CAM_MODE_SECURITY       4  // Security camera style
#define CAM_MODE_DRAMATIC       5  // Over-the-shoulder dramatic angles

// Transition types
#define CAM_TRANSITION_LINEAR     0
#define CAM_TRANSITION_SMOOTH     1
#define CAM_TRANSITION_CINEMATIC  2
#define CAM_TRANSITION_CUT        3

// Global variables
extern vec3_t cam_ofs;
extern int cam_thirdperson;
extern camera_state_t g_camera_state;

// =======================================================
//          FUNCTION DECLARATIONS
// =======================================================

// Core functions
void CAM_Init(void);
void CAM_ClearStates(void);

// Camera mode switching
void CAM_ToFirstPerson(void);
void CAM_ToThirdPerson(void);

// Input handling functions called by engine commands
void CAM_StartMouseMove(void);
void CAM_EndMouseMove(void);
void CAM_StartDistance(void);
void CAM_EndDistance(void);

// Enhanced survival horror functions
void CAM_InitSurvivalHorror(void);
void CAM_UpdateSurvivalHorror(void);
void CAM_LoadCameraZones(const char *mapname);
void CAM_SetCameraMode(int mode);
void CAM_SetTensionLevel(float level);
void CAM_TriggerCinematic(int sequence_id);
void CAM_AddAtmosphericShake(float intensity, float duration);
void CAM_SetColorCorrection(float r, float g, float b);
void CAM_EnableVignette(float intensity);

// Zone management
int CAM_AddCameraZone(vec3_t mins, vec3_t maxs, vec3_t cam_pos, vec3_t cam_angles);
void CAM_UpdateCameraZones(void);
int CAM_GetPlayerZone(void);
void CAM_TransitionToZone(int zone_id);

// Cinematic system
int CAM_CreateCinematicSequence(void);
void CAM_AddKeyframe(int sequence_id, float time, vec3_t pos, vec3_t angles, float fov);
void CAM_PlayCinematicSequence(int sequence_id);
void CAM_StopCinematicSequence(void);

// Camera calculation functions
void CAM_CalculateFixedAngleCamera(void);
void CAM_UpdateCinematicSequence(void);
void CAM_CalculateDramaticCamera(void);
void CAM_CalculateSecurityCamera(void);
void CAM_CalculateCrosshairWorldPosition(vec3_t view_origin, vec3_t view_angles);
void CAM_OptimizeCameraForCrosshair(vec3_t player_origin, vec3_t base_cam_pos, vec3_t *final_cam_pos, vec3_t *final_cam_angles);

// Utility functions
float MoveToward(float cur, float goal, float maxspeed);
float VectorLength(vec3_t v);
bool CAM_TraceCollision(vec3_t start, vec3_t end, vec3_t result);
void CAM_SmoothTransition(vec3_t current, vec3_t target, float speed);
void CAM_CalculateAtmosphericEffects(void);
void CAM_ApplyScreenEffects(void);
bool CAM_IsPlayerInZone(int zone_id);
float CAM_MoveTowardTactical(float cur, float goal, float maxspeed);

float CAM_EaseInOut(float t);
float CAM_EaseIn(float t);
float CAM_EaseOut(float t);

void CAM_UpdateCVARs(void);

// Engine-facing functions exported from in_camera.cpp
extern "C"
{
    void DLLEXPORT CAM_Think(void);
    int DLLEXPORT CL_IsThirdPerson(void);
    void DLLEXPORT CL_CameraOffset(float *ofs);
}

#endif // _CAMERA_H_