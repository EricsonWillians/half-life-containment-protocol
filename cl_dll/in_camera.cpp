//========= Copyright (c) 1996-2002, Valve LLC, All rights reserved. ============
//
// Purpose: Enhanced Survival Horror Camera System with SUPERIOR Third Person Experience
//          Optimized for Horror Shooter with Always-Visible Crosshair
//
// $NoKeywords: $
//=============================================================================

#include "hud.h"
#include "cl_util.h"
#include "camera.h"
#include "kbutton.h"
#include "cvardef.h"
#include "usercmd.h"
#include "const.h"
#include "in_defs.h"
#include "pmtrace.h"
#include "pm_defs.h"
#include <math.h>
#include "event_api.h"

#if XASH_WIN32
#define WIN32_LEAN_AND_MEAN
#define WIN32_EXTRA_LEAN
#define HSPRITE WINDOWS_HSPRITE
#include <windows.h>
#undef HSPRITE
#else
typedef struct point_s
{
    int x;
    int y;
} POINT;
#define GetCursorPos(x)
#define SetCursorPos(x,y)
#endif

float CL_KeyState(kbutton_t *key);

extern cl_enginefunc_t gEngfuncs;

//-------------------------------------------------- ENHANCED CONSTANTS FOR PRODUCTION QUALITY

// Core third person positioning for SUPERIOR crosshair visibility
#define CAM_SH_OPTIMAL_DISTANCE         80.0f   // Perfect tactical distance
#define CAM_SH_CROSSHAIR_OFFSET_RIGHT   28.0f   // Over-the-shoulder offset for unobstructed aim
#define CAM_SH_CROSSHAIR_OFFSET_UP      12.0f   // Height offset for clear sight lines
#define CAM_SH_MIN_DISTANCE             24.0f   // Minimum distance (tighter than default)
#define CAM_SH_MAX_DISTANCE             180.0f  // Maximum useful distance

// Enhanced smoothing and responsiveness
#define CAM_SH_SMOOTH_FACTOR            0.08f   // Professional-grade smoothing
#define CAM_SH_ANGLE_SPEED              3.2f    // Responsive but smooth
#define CAM_SH_TRANSITION_SPEED         4.5f    // Fast transitions for action
#define CAM_SH_COLLISION_SPEED          6.0f    // Quick collision recovery

// Mathematical precision and safety
#define CAM_SH_EPSILON                  0.001f  // Floating point precision
#define CAM_SH_COLLISION_RADIUS         16.0f   // Safe collision buffer
#define CAM_SH_MAX_TRACE_DIST           1024.0f // Maximum trace distance
#define CAM_SH_ANGLE_WRAP_THRESHOLD     270.0f  // Angle wrapping threshold

// Horror atmosphere constants
#define CAM_SH_SHAKE_DECAY              0.92f   // Natural shake decay
#define CAM_SH_SWAY_FREQUENCY           0.3f    // Subtle atmospheric movement
#define CAM_SH_SWAY_AMPLITUDE           0.12f   // Minimal sway for immersion
#define CAM_SH_TENSION_SCALE            1.4f    // Tension effect multiplier

// Crosshair optimization
#define CAM_SH_CROSSHAIR_PRIORITY       2.0f    // High priority for crosshair visibility
#define CAM_SH_AIM_TRACE_DIST           256.0f  // Distance for aim tracing
#define CAM_SH_VISIBILITY_THRESHOLD     0.75f   // Minimum acceptable visibility

// Original constants (preserved for compatibility)
#define CAM_DIST_DELTA                  1.0f
#define CAM_ANGLE_DELTA                 2.5f
#define CAM_ANGLE_SPEED                 2.5f
#define CAM_MIN_DIST                    30.0f
#define CAM_ANGLE_MOVE                  0.5f
#define MAX_ANGLE_DIFF                  10.0f
#define PITCH_MAX                       90.0f
#define PITCH_MIN                       0.0f
#define YAW_MAX                         135.0f
#define YAW_MIN                         -135.0f

enum ECAM_Command
{
    CAM_COMMAND_NONE = 0,
    CAM_COMMAND_TOTHIRDPERSON = 1,
    CAM_COMMAND_TOFIRSTPERSON = 2,
    CAM_COMMAND_TOFIXEDANGLE = 3,
    CAM_COMMAND_TOCINEMATIC = 4,
    CAM_COMMAND_TODRAMATIC = 5,
    CAM_COMMAND_TOSECURITY = 6
};

//-------------------------------------------------- Global Variables

// Original CVars
cvar_t *cam_command;
cvar_t *cam_snapto;
cvar_t *cam_idealyaw;
cvar_t *cam_idealpitch;
cvar_t *cam_idealdist;
cvar_t *cam_contain;
cvar_t *c_maxpitch;
cvar_t *c_minpitch;
cvar_t *c_maxyaw;
cvar_t *c_minyaw;
cvar_t *c_maxdistance;
cvar_t *c_mindistance;

// Enhanced Survival Horror CVars
cvar_t *cam_sh_enable;
cvar_t *cam_sh_mode;
cvar_t *cam_sh_transition_speed;
cvar_t *cam_sh_atmospheric_shake;
cvar_t *cam_sh_tension_level;
cvar_t *cam_sh_smooth_movement;
cvar_t *cam_sh_collision_detection;
cvar_t *cam_sh_dramatic_angles;
cvar_t *cam_sh_vignette;
cvar_t *cam_sh_color_correction;
cvar_t *cam_sh_audio_reactive;
cvar_t *cam_sh_auto_zones;
cvar_t *cam_sh_cinematic_fov;
cvar_t *cam_sh_security_mode;
cvar_t *cam_sh_follow_speed;
cvar_t *cam_sh_look_ahead;
cvar_t *cam_sh_height_offset;
cvar_t *cam_sh_distance_scale;
cvar_t *cam_sh_angle_constraints;

// Crosshair enhancement CVars  
cvar_t *cam_sh_crosshair_priority;
cvar_t *cam_sh_shoulder_offset;
cvar_t *cam_sh_aim_assist;
cvar_t *cam_sh_auto_center;

// Camera state
camera_state_t g_camera_state;
vec3_t cam_ofs;
int cam_thirdperson;
int cam_mousemove = 0;
int iMouseInUse = 0;
int cam_distancemove;
extern int mouse_x, mouse_y;
int cam_old_mouse_x, cam_old_mouse_y;
POINT cam_mouse;

// Timing and effects
float cam_last_time = 0.0f;
float cam_frame_time = 0.0f;
float cam_shake_time = 0.0f;
float cam_shake_intensity = 0.0f;
vec3_t cam_atmospheric_offset = {0, 0, 0};
float cam_sway_time = 0.0f;
int cam_last_zone = -1;
float cam_zone_transition_time = 0.0f;

//-------------------------------------------------- Local Variables

static kbutton_t cam_pitchup, cam_pitchdown, cam_yawleft, cam_yawright;
static kbutton_t cam_in, cam_out, cam_move;

//-------------------------------------------------- PRODUCTION-GRADE UTILITY FUNCTIONS

float VectorLength(vec3_t v)
{
    float length_sq = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
    if (length_sq < CAM_SH_EPSILON)
        return 0.0f;
    return sqrt(length_sq);
}

float CAM_ClampFloat(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

float CAM_NormalizeAngle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

float CAM_AngleDifference(float a1, float a2)
{
    float diff = a2 - a1;
    return CAM_NormalizeAngle(diff);
}

bool CAM_IsValidFloat(float f)
{
    return !isnan(f) && isfinite(f);
}

bool CAM_IsValidVector(vec3_t v)
{
    return CAM_IsValidFloat(v[0]) && CAM_IsValidFloat(v[1]) && CAM_IsValidFloat(v[2]);
}

void CAM_CopyValidVector(vec3_t src, vec3_t dst)
{
    if (CAM_IsValidVector(src))
    {
        VectorCopy(src, dst);
    }
    else
    {
        VectorClear(dst);
    }
}

float CAM_EaseInOut(float t)
{
    t = CAM_ClampFloat(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float CAM_EaseIn(float t)
{
    t = CAM_ClampFloat(t, 0.0f, 1.0f);
    return t * t;
}

float CAM_EaseOut(float t)
{
    t = CAM_ClampFloat(t, 0.0f, 1.0f);
    return 1.0f - (1.0f - t) * (1.0f - t);
}

//-------------------------------------------------- COLLISION AND TRACE SYSTEM

bool CAM_TraceCollision(vec3_t start, vec3_t end, vec3_t result)
{
    if (!CAM_IsValidVector(start) || !CAM_IsValidVector(end))
    {
        VectorCopy(start, result);
        return false;
    }
    
    vec3_t trace_vec;
    VectorSubtract(end, start, trace_vec);
    float trace_dist = VectorLength(trace_vec);
    
    if (trace_dist > CAM_SH_MAX_TRACE_DIST)
    {
        VectorNormalize(trace_vec);
        VectorMA(start, CAM_SH_MAX_TRACE_DIST, trace_vec, end);
    }
    
    pmtrace_t trace;

    gEngfuncs.pEventAPI->EV_SetUpPlayerPrediction(false, true);    
    gEngfuncs.pEventAPI->EV_PushPMStates();
    gEngfuncs.pEventAPI->EV_SetSolidPlayers(-1);
    gEngfuncs.pEventAPI->EV_SetTraceHull(2);
    gEngfuncs.pEventAPI->EV_PlayerTrace(start, end, PM_GLASS_IGNORE | PM_STUDIO_BOX, gEngfuncs.GetLocalPlayer()->index, &trace);
    gEngfuncs.pEventAPI->EV_PopPMStates();

    if (trace.fraction < 1.0f)
    {
        vec3_t safe_pos;
        VectorNormalize(trace_vec);
        VectorMA(trace.endpos, -CAM_SH_COLLISION_RADIUS, trace_vec, safe_pos);
        VectorCopy(safe_pos, result);
        return true;
    }
    else
    {
        VectorCopy(trace.endpos, result);
        return false;
    }
}

//-------------------------------------------------- CROSSHAIR OPTIMIZATION SYSTEM

void CAM_CalculateCrosshairWorldPosition(vec3_t player_origin, vec3_t view_angles)
{
    if (!CAM_IsValidVector(player_origin) || !CAM_IsValidVector(view_angles))
    {
        VectorClear(g_camera_state.crosshair_world_pos);
        return;
    }
    
    vec3_t forward, right, up;
    AngleVectors(view_angles, forward, right, up);
    
    vec3_t crosshair_pos;
    VectorMA(player_origin, CAM_SH_AIM_TRACE_DIST, forward, crosshair_pos);
    
    vec3_t trace_result;
    if (CAM_TraceCollision(player_origin, crosshair_pos, trace_result))
    {
        VectorCopy(trace_result, g_camera_state.crosshair_world_pos);
        g_camera_state.crosshair_obstructed = true;
    }
    else
    {
        VectorCopy(crosshair_pos, g_camera_state.crosshair_world_pos);
        g_camera_state.crosshair_obstructed = false;
    }
}

void CAM_OptimizeCameraForCrosshair(vec3_t player_origin, vec3_t view_angles, vec3_t *optimal_pos, vec3_t *optimal_angles)
{
    if (!CAM_IsValidVector(player_origin) || !CAM_IsValidVector(view_angles))
        return;
    
    vec3_t forward, right, up;
    AngleVectors(view_angles, forward, right, up);
    
    // Calculate enhanced shoulder positioning
    float shoulder_offset = CAM_SH_CROSSHAIR_OFFSET_RIGHT;
	float height_offset = CAM_SH_CROSSHAIR_OFFSET_UP;
	float optimal_distance = CAM_SH_OPTIMAL_DISTANCE;

	if (cam_sh_shoulder_offset && cam_sh_shoulder_offset->value != 0.0f)
	{
		shoulder_offset *= CAM_ClampFloat(cam_sh_shoulder_offset->value, 0.1f, 3.0f);
	}

	if (cam_sh_height_offset && cam_sh_height_offset->value != 0.0f)
	{
		height_offset *= CAM_ClampFloat(cam_sh_height_offset->value, 0.1f, 3.0f);
	}
    
    // Calculate optimal camera position for crosshair visibility
    vec3_t base_cam_pos;
    VectorMA(player_origin, -optimal_distance, forward, base_cam_pos);
    VectorMA(base_cam_pos, shoulder_offset, right, base_cam_pos);
    VectorMA(base_cam_pos, height_offset, up, base_cam_pos);
    
    // Check crosshair visibility from this position
    vec3_t camera_to_crosshair;
    VectorSubtract(g_camera_state.crosshair_world_pos, base_cam_pos, camera_to_crosshair);
    VectorNormalize(camera_to_crosshair);
    
    vec3_t player_to_crosshair;
    VectorSubtract(g_camera_state.crosshair_world_pos, player_origin, player_to_crosshair);
    VectorNormalize(player_to_crosshair);
    
    float visibility_dot = DotProduct(camera_to_crosshair, player_to_crosshair);
    g_camera_state.crosshair_visibility = CAM_ClampFloat((visibility_dot + 1.0f) * 0.5f, 0.0f, 1.0f);
    
    // Adjust position if crosshair visibility is poor
    if (g_camera_state.crosshair_visibility < CAM_SH_VISIBILITY_THRESHOLD)
    {
        vec3_t adjustment;
        VectorScale(right, shoulder_offset * 0.3f, adjustment);
        VectorAdd(base_cam_pos, adjustment, base_cam_pos);
        
        // Recalculate visibility
        VectorSubtract(g_camera_state.crosshair_world_pos, base_cam_pos, camera_to_crosshair);
        VectorNormalize(camera_to_crosshair);
        visibility_dot = DotProduct(camera_to_crosshair, player_to_crosshair);
        g_camera_state.crosshair_visibility = CAM_ClampFloat((visibility_dot + 1.0f) * 0.5f, 0.0f, 1.0f);
    }
    
    // Output results
    if (optimal_pos && CAM_IsValidVector(base_cam_pos))
    {
        VectorCopy(base_cam_pos, *optimal_pos);
    }
    
    if (optimal_angles && CAM_IsValidVector(view_angles))
    {
        VectorCopy(view_angles, *optimal_angles);
    }
}

//-------------------------------------------------- ATMOSPHERIC SYSTEM

void CAM_CalculateAtmosphericEffects(void)
{
    float current_time = gEngfuncs.GetClientTime();
    cam_frame_time = current_time - cam_last_time;
    cam_last_time = current_time;
    
    cam_frame_time = CAM_ClampFloat(cam_frame_time, 0.0f, 0.1f);
    
    // Enhanced shake system
    if (cam_shake_time > 0.0f)
    {
        cam_shake_time -= cam_frame_time;
        cam_shake_intensity *= CAM_SH_SHAKE_DECAY;
        
        if (cam_shake_time <= 0.0f || cam_shake_intensity < CAM_SH_EPSILON)
        {
            cam_shake_intensity = 0.0f;
            VectorClear(g_camera_state.shake_offset);
        }
        else
        {
            // Reduce shake when aiming for precision
            float shake_reduction = 1.0f;
            if (cam_sh_crosshair_priority && cam_sh_crosshair_priority->value > 0.0f)
            {
                shake_reduction = 1.0f - (cam_sh_crosshair_priority->value * 0.4f);
            }
            
            g_camera_state.shake_offset[0] = gEngfuncs.pfnRandomFloat(-1.0f, 1.0f) * cam_shake_intensity * shake_reduction;
            g_camera_state.shake_offset[1] = gEngfuncs.pfnRandomFloat(-1.0f, 1.0f) * cam_shake_intensity * shake_reduction;
            g_camera_state.shake_offset[2] = gEngfuncs.pfnRandomFloat(-1.0f, 1.0f) * cam_shake_intensity * shake_reduction * 0.3f;
        }
    }
    
    // Atmospheric sway
    cam_sway_time += cam_frame_time * CAM_SH_SWAY_FREQUENCY;
    float atmospheric_value = 0.0f;
    if (cam_sh_atmospheric_shake)
    {
        atmospheric_value = CAM_ClampFloat(cam_sh_atmospheric_shake->value, 0.0f, 1.0f);
    }
    
    if (atmospheric_value > CAM_SH_EPSILON)
    {
        // Reduced sway when aiming
        float sway_reduction = 1.0f;
        if (cam_sh_crosshair_priority && cam_sh_crosshair_priority->value > 0.0f)
        {
            sway_reduction = 1.0f - (cam_sh_crosshair_priority->value * 0.3f);
        }
        
        g_camera_state.sway_offset[0] = sin(cam_sway_time * 0.7f) * CAM_SH_SWAY_AMPLITUDE * atmospheric_value * sway_reduction;
        g_camera_state.sway_offset[1] = cos(cam_sway_time * 0.5f) * CAM_SH_SWAY_AMPLITUDE * atmospheric_value * sway_reduction;
        g_camera_state.sway_offset[2] = sin(cam_sway_time * 0.3f) * CAM_SH_SWAY_AMPLITUDE * atmospheric_value * sway_reduction * 0.2f;
    }
    else
    {
        VectorClear(g_camera_state.sway_offset);
    }
    
    // Tension effects
    float tension = 0.0f;
    if (cam_sh_tension_level)
    {
        tension = CAM_ClampFloat(cam_sh_tension_level->value, 0.0f, 1.0f);
    }
    g_camera_state.tension_level = tension;
    g_camera_state.atmospheric_shake = tension * CAM_SH_TENSION_SCALE;
}

//-------------------------------------------------- ENHANCED CAM_Think - THE CORE FUNCTION

void DLLEXPORT CAM_Think(void)
{
    CAM_UpdateCVARs();

    vec3_t origin;
    vec3_t ext, pnt, camForward, camRight, camUp;
    float dist;
    vec3_t camAngles;
    float flSensitivity;
    vec3_t viewangles;
    vec3_t player_origin;

    // Get critical player information with validation
    VectorCopy(gEngfuncs.GetLocalPlayer()->origin, player_origin);
    gEngfuncs.GetViewAngles((float *)viewangles);

    if (!CAM_IsValidVector(player_origin) || !CAM_IsValidVector(viewangles))
    {
        VectorClear(player_origin);
        VectorClear(viewangles);
    }

    // Multiplayer restriction
    if (gEngfuncs.GetMaxClients() > 1 && CL_IsThirdPerson())
        CAM_ToFirstPerson();

    // Process camera commands
    int cam_cmd = (int)cam_command->value;
    switch (cam_cmd)
    {
        case CAM_COMMAND_TOTHIRDPERSON:
            CAM_ToThirdPerson();
            break;
        case CAM_COMMAND_TOFIRSTPERSON:
            CAM_ToFirstPerson();
            break;
        case CAM_COMMAND_TOFIXEDANGLE:
            CAM_SetCameraMode(CAM_MODE_FIXEDANGLE);
            break;
        case CAM_COMMAND_TOCINEMATIC:
            CAM_SetCameraMode(CAM_MODE_CINEMATIC);
            break;
        case CAM_COMMAND_TODRAMATIC:
            CAM_SetCameraMode(CAM_MODE_DRAMATIC);
            break;
        case CAM_COMMAND_TOSECURITY:
            CAM_SetCameraMode(CAM_MODE_SECURITY);
            break;
        default:
            break;
    }

    // Early exit for first person
    if (!cam_thirdperson && g_camera_state.mode == CAM_MODE_FIRSTPERSON)
        return;

    // Update enhanced camera system
    bool enhanced_mode = (cam_sh_enable && cam_sh_enable->value);
    if (enhanced_mode)
    {
        CAM_CalculateAtmosphericEffects();
        CAM_CalculateCrosshairWorldPosition(player_origin, viewangles);
        
        // Handle special camera modes
        switch (g_camera_state.mode)
        {
            case CAM_MODE_FIXEDANGLE:
                CAM_CalculateFixedAngleCamera();
                return;
            case CAM_MODE_CINEMATIC:
                CAM_UpdateCinematicSequence();
                return;
            case CAM_MODE_DRAMATIC:
                CAM_CalculateDramaticCamera();
                return;
            case CAM_MODE_SECURITY:
                CAM_CalculateSecurityCamera();
                return;
            default:
                break;
        }
    }

    // =====================================================================
    // SUPERIOR THIRD PERSON CAMERA WITH TACTICAL OVER-THE-SHOULDER POSITIONING
    // =====================================================================

    // Initialize camera angles with enhanced validation
    camAngles[PITCH] = CAM_ClampFloat(cam_idealpitch->value, -89.0f, 89.0f);
    camAngles[YAW] = CAM_NormalizeAngle(cam_idealyaw->value);
    
    // Enhanced distance calculation with crosshair optimization
    float base_distance = CAM_ClampFloat(cam_idealdist->value, CAM_SH_MIN_DISTANCE, CAM_SH_MAX_DISTANCE);
    
    // Crosshair-optimized distance
    if (enhanced_mode && cam_sh_crosshair_priority && cam_sh_crosshair_priority->value != 0.0f)
    {
        float crosshair_factor = CAM_ClampFloat(cam_sh_crosshair_priority->value, 0.0f, 2.0f);
        base_distance = base_distance * (1.0f - crosshair_factor * 0.2f) + CAM_SH_OPTIMAL_DISTANCE * (crosshair_factor * 0.5f);
    }
    
    dist = base_distance;

    // ENHANCED MOUSE MOVEMENT with crosshair-aware sensitivity
    if (cam_mousemove)
    {
        GetCursorPos(&cam_mouse);

        if (!cam_distancemove)
        {
            int center_x = gEngfuncs.GetWindowCenterX();
            int center_y = gEngfuncs.GetWindowCenterY();
            
            // Enhanced sensitivity system
            float base_sensitivity = 1.0f;
            if (enhanced_mode && cam_sh_crosshair_priority && cam_sh_crosshair_priority->value > 0.0f)
            {
                // Improved sensitivity when crosshair is visible
                base_sensitivity *= (1.0f + g_camera_state.crosshair_visibility * 0.25f);
            }
            
            float tension_factor = 1.0f + g_camera_state.tension_level * 0.15f;
            float yaw_sensitivity = base_sensitivity * tension_factor;
            float pitch_sensitivity = base_sensitivity * tension_factor * 0.85f;
            
            // Enhanced YAW control with better responsiveness
            float max_yaw = CAM_ClampFloat(c_maxyaw->value, -180.0f, 180.0f);
            float min_yaw = CAM_ClampFloat(c_minyaw->value, -180.0f, 180.0f);
            
            // Extended yaw range for better tactical positioning
            if (enhanced_mode && cam_sh_angle_constraints && cam_sh_angle_constraints->value)
            {
                max_yaw = CAM_ClampFloat(max_yaw * 1.15f, -180.0f, 180.0f);
                min_yaw = CAM_ClampFloat(min_yaw * 1.15f, -180.0f, 180.0f);
            }
            
            if (cam_mouse.x > center_x)
            {
                float delta = CAM_ANGLE_MOVE * yaw_sensitivity * ((cam_mouse.x - center_x) * 0.25f);
                delta = CAM_ClampFloat(delta, 0.0f, 5.0f);
                
                if (camAngles[YAW] < max_yaw)
                {
                    camAngles[YAW] += delta;
                    if (camAngles[YAW] > max_yaw)
                        camAngles[YAW] = max_yaw;
                }
            }
            else if (cam_mouse.x < center_x)
            {
                float delta = CAM_ANGLE_MOVE * yaw_sensitivity * ((center_x - cam_mouse.x) * 0.25f);
                delta = CAM_ClampFloat(delta, 0.0f, 5.0f);
                
                if (camAngles[YAW] > min_yaw)
                {
                    camAngles[YAW] -= delta;
                    if (camAngles[YAW] < min_yaw)
                        camAngles[YAW] = min_yaw;
                }
            }

            // Enhanced PITCH control with crosshair optimization
            float max_pitch = CAM_ClampFloat(c_maxpitch->value, 0.0f, 89.0f);
            float min_pitch = CAM_ClampFloat(c_minpitch->value, -89.0f, 0.0f);
            
            // Extended pitch range for better aiming angles
            if (enhanced_mode && cam_sh_dramatic_angles && cam_sh_dramatic_angles->value)
            {
                max_pitch = CAM_ClampFloat(max_pitch + 20.0f, 0.0f, 89.0f);
                min_pitch = CAM_ClampFloat(min_pitch - 15.0f, -89.0f, 0.0f);
            }
            
            if (cam_mouse.y > center_y)
            {
                float delta = CAM_ANGLE_MOVE * pitch_sensitivity * ((cam_mouse.y - center_y) * 0.25f);
                delta = CAM_ClampFloat(delta, 0.0f, 3.0f);
                
                if (camAngles[PITCH] < max_pitch)
                {
                    camAngles[PITCH] += delta;
                    if (camAngles[PITCH] > max_pitch)
                        camAngles[PITCH] = max_pitch;
                }
            }
            else if (cam_mouse.y < center_y)
            {
                float delta = CAM_ANGLE_MOVE * pitch_sensitivity * ((center_y - cam_mouse.y) * 0.25f);
                delta = CAM_ClampFloat(delta, 0.0f, 3.0f);
                
                if (camAngles[PITCH] > min_pitch)
                {
                    camAngles[PITCH] -= delta;
                    if (camAngles[PITCH] < min_pitch)
                        camAngles[PITCH] = min_pitch;
                }
            }

            // Store mouse state
            flSensitivity = gHUD.GetSensitivity();
            if (flSensitivity > CAM_SH_EPSILON)
            {
                cam_old_mouse_x = cam_mouse.x * flSensitivity;
                cam_old_mouse_y = cam_mouse.y * flSensitivity;
            }
            else
            {
                cam_old_mouse_x = cam_mouse.x;
                cam_old_mouse_y = cam_mouse.y;
            }
            SetCursorPos(center_x, center_y);
        }
    }

    // ENHANCED KEYBOARD CONTROLS with tactical responsiveness
    float angle_delta = 2.0f;
    float dist_delta = 2.5f;
    
    // Crosshair-aware control enhancement
    if (enhanced_mode && cam_sh_crosshair_priority && cam_sh_crosshair_priority->value > 0.0f)
    {
        if (g_camera_state.crosshair_visibility < CAM_SH_VISIBILITY_THRESHOLD)
        {
            angle_delta *= 1.4f; // Faster movement when crosshair is obstructed
            dist_delta *= 1.3f;
        }
    }
    
    // Tension-based control adjustment
    float tension_multiplier = 1.0f + g_camera_state.tension_level * 0.3f;
    angle_delta *= tension_multiplier;
    dist_delta *= tension_multiplier;
    
    angle_delta = CAM_ClampFloat(angle_delta, 0.5f, 4.0f);
    dist_delta = CAM_ClampFloat(dist_delta, 0.5f, 7.0f);
    
    if (CL_KeyState(&cam_pitchup))
        camAngles[PITCH] += angle_delta;
    else if (CL_KeyState(&cam_pitchdown))
        camAngles[PITCH] -= angle_delta;

    if (CL_KeyState(&cam_yawleft))
        camAngles[YAW] -= angle_delta;
    else if (CL_KeyState(&cam_yawright))
        camAngles[YAW] += angle_delta;

    // Clamp angles after keyboard input
    camAngles[PITCH] = CAM_ClampFloat(camAngles[PITCH], -89.0f, 89.0f);
    camAngles[YAW] = CAM_NormalizeAngle(camAngles[YAW]);

    // SUPERIOR DISTANCE CONTROL with crosshair optimization
    float distance_scale = 1.0f;
    if (cam_sh_distance_scale)
    {
        distance_scale = CAM_ClampFloat(cam_sh_distance_scale->value, 0.3f, 3.0f);
    }
    
    float min_dist = CAM_SH_MIN_DISTANCE * distance_scale;
    float max_dist = CAM_SH_MAX_DISTANCE * distance_scale;
    
    // Crosshair-optimized distance bounds
    if (enhanced_mode && cam_sh_crosshair_priority && cam_sh_crosshair_priority->value > 0.0f)
    {
        float crosshair_factor = CAM_ClampFloat(cam_sh_crosshair_priority->value, 0.0f, 1.0f);
        float optimal_min = CAM_SH_OPTIMAL_DISTANCE * 0.5f;
        float optimal_max = CAM_SH_OPTIMAL_DISTANCE * 2.0f;
        
        min_dist = min_dist * (1.0f - crosshair_factor) + optimal_min * crosshair_factor;
        max_dist = max_dist * (1.0f - crosshair_factor) + optimal_max * crosshair_factor;
    }
    
    if (CL_KeyState(&cam_in))
    {
        dist -= dist_delta;
        if (dist < min_dist)
        {
            if (dist < CAM_MIN_DIST)
            {
                camAngles[PITCH] = 0;
                camAngles[YAW] = 0;
                dist = CAM_MIN_DIST;
            }
            else
            {
                dist = min_dist;
            }
        }
    }
    else if (CL_KeyState(&cam_out))
    {
        dist += dist_delta;
        if (dist > max_dist)
            dist = max_dist;
    }

    // Enhanced distance mouse control
    if (cam_distancemove)
    {
        int center_y = gEngfuncs.GetWindowCenterY();
        
        if (cam_mouse.y > center_y)
        {
            if (dist < max_dist)
            {
                float delta = dist_delta * CAM_ClampFloat((cam_mouse.y - center_y) * 0.3f, 0.0f, 10.0f);
                dist += delta;
            }
            if (dist > max_dist)
                dist = max_dist;
        }
        else if (cam_mouse.y < center_y)
        {
            if (dist > min_dist)
            {
                float delta = dist_delta * CAM_ClampFloat((center_y - cam_mouse.y) * 0.3f, 0.0f, 10.0f);
                dist -= delta;
            }
            if (dist < min_dist)
                dist = min_dist;
        }
        
        flSensitivity = gHUD.GetSensitivity();
        cam_old_mouse_x = cam_mouse.x * flSensitivity;
        cam_old_mouse_y = cam_mouse.y * flSensitivity;
        SetCursorPos(gEngfuncs.GetWindowCenterX(), gEngfuncs.GetWindowCenterY());
    }

    // Final distance validation
    dist = CAM_ClampFloat(dist, min_dist, max_dist);

    // Update ideal values
    cam_idealpitch->value = CAM_ClampFloat(camAngles[PITCH], -89.0f, 89.0f);
    cam_idealyaw->value = CAM_NormalizeAngle(camAngles[YAW]);
    cam_idealdist->value = dist;

    // =====================================================================
    // TACTICAL OVER-THE-SHOULDER CAMERA POSITIONING - THE CORE ENHANCEMENT
    // =====================================================================

    // Calculate base camera position using view angles (player look direction)
    vec3_t camera_angles;
    VectorCopy(viewangles, camera_angles);
    
    // Apply camera offset angles for over-the-shoulder effect
    camera_angles[YAW] += cam_idealyaw->value;
    camera_angles[PITCH] += cam_idealpitch->value;
    
    // Clamp final camera angles
    camera_angles[PITCH] = CAM_ClampFloat(camera_angles[PITCH], -89.0f, 89.0f);
    camera_angles[YAW] = CAM_NormalizeAngle(camera_angles[YAW]);
    camera_angles[ROLL] = 0.0f; // No roll for tactical cameras
    
    // Calculate direction vectors from camera angles
    AngleVectors(camera_angles, camForward, camRight, camUp);
    
    // TACTICAL SHOULDER OFFSET CALCULATION
    float shoulder_offset_right = CAM_SH_CROSSHAIR_OFFSET_RIGHT;
    float shoulder_offset_up = CAM_SH_CROSSHAIR_OFFSET_UP;
    
    // Apply shoulder offset scaling
    if (enhanced_mode && cam_sh_shoulder_offset && cam_sh_shoulder_offset->value != 0.0f)
    {
        float offset_scale = CAM_ClampFloat(cam_sh_shoulder_offset->value, 0.1f, 3.0f);
        shoulder_offset_right *= offset_scale;
    }
    
    if (enhanced_mode && cam_sh_height_offset && cam_sh_height_offset->value != 0.0f)
    {
        float height_scale = CAM_ClampFloat(cam_sh_height_offset->value, 0.1f, 3.0f);
        shoulder_offset_up *= height_scale;
    }
    
    // Calculate optimal camera position
    vec3_t ideal_camera_pos;
    
    // Start from player position
    VectorCopy(player_origin, ideal_camera_pos);
    
    // Move back along camera forward direction
    VectorMA(ideal_camera_pos, -dist, camForward, ideal_camera_pos);
    
    // Apply tactical over-the-shoulder offset
    VectorMA(ideal_camera_pos, shoulder_offset_right, camRight, ideal_camera_pos);
    VectorMA(ideal_camera_pos, shoulder_offset_up, camUp, ideal_camera_pos);
    
    // PROFESSIONAL COLLISION DETECTION with crosshair preservation
    vec3_t final_camera_pos;
    VectorCopy(ideal_camera_pos, final_camera_pos);
    
    if (enhanced_mode && cam_sh_collision_detection && cam_sh_collision_detection->value)
    {
        vec3_t collision_result;
        if (CAM_TraceCollision(player_origin, ideal_camera_pos, collision_result))
        {
            // Calculate safe position maintaining shoulder offset
            vec3_t collision_dir;
            VectorSubtract(collision_result, player_origin, collision_dir);
            float collision_dist = VectorLength(collision_dir);
            VectorNormalize(collision_dir);
            
            // Maintain minimum distance for crosshair functionality
            float safe_dist = collision_dist - CAM_SH_COLLISION_RADIUS;
            if (enhanced_mode && cam_sh_crosshair_priority && cam_sh_crosshair_priority->value > 0.0f)
            {
                float min_crosshair_dist = CAM_SH_OPTIMAL_DISTANCE * 0.4f;
                if (safe_dist < min_crosshair_dist)
                {
                    safe_dist = min_crosshair_dist;
                }
            }
            
            safe_dist = CAM_ClampFloat(safe_dist, min_dist * 0.5f, dist);
            
            // Recalculate position with safe distance
            VectorMA(player_origin, -safe_dist, camForward, final_camera_pos);
            VectorMA(final_camera_pos, shoulder_offset_right, camRight, final_camera_pos);
            VectorMA(final_camera_pos, shoulder_offset_up, camUp, final_camera_pos);
            
            // Update distance for smooth interpolation
            dist = safe_dist;
            cam_idealdist->value = dist;
        }
    }

    // PROFESSIONAL-GRADE SMOOTH MOVEMENT
    VectorCopy(cam_ofs, camAngles);

    float transition_speed = CAM_SH_TRANSITION_SPEED;
    if (cam_sh_transition_speed)
    {
        transition_speed = CAM_SH_TRANSITION_SPEED * CAM_ClampFloat(cam_sh_transition_speed->value, 0.1f, 10.0f);
    }
    
    // Faster transitions when crosshair visibility is poor
    if (enhanced_mode && g_camera_state.crosshair_visibility < CAM_SH_VISIBILITY_THRESHOLD)
    {
        transition_speed *= 1.8f;
    }
    
    if (cam_snapto->value)
    {
        // Immediate positioning for snap mode
        camAngles[YAW] = CAM_NormalizeAngle(cam_idealyaw->value + viewangles[YAW]);
        camAngles[PITCH] = CAM_ClampFloat(cam_idealpitch->value + viewangles[PITCH], -89.0f, 89.0f);
        camAngles[2] = cam_idealdist->value;
    }
    else
    {
        // ENHANCED smooth movement with angle wrapping
        float target_yaw = CAM_NormalizeAngle(cam_idealyaw->value + viewangles[YAW]);
        float target_pitch = CAM_ClampFloat(cam_idealpitch->value + viewangles[PITCH], -89.0f, 89.0f);
        
        // Superior yaw smoothing
        float yaw_diff = CAM_AngleDifference(camAngles[YAW] - viewangles[YAW], cam_idealyaw->value);
        if (fabs(yaw_diff) > CAM_SH_EPSILON)
        {
            if (fabs(yaw_diff) > CAM_SH_ANGLE_WRAP_THRESHOLD)
            {
                if (target_yaw > camAngles[YAW])
                    target_yaw -= 360.0f;
                else
                    target_yaw += 360.0f;
            }
            camAngles[YAW] = MoveToward(camAngles[YAW], target_yaw, transition_speed);
        }

        // Superior pitch smoothing
        float pitch_diff = (camAngles[PITCH] - viewangles[PITCH]) - cam_idealpitch->value;
        if (fabs(pitch_diff) > CAM_SH_EPSILON)
        {
            camAngles[PITCH] = MoveToward(camAngles[PITCH], target_pitch, transition_speed);
        }

        // Enhanced distance smoothing
        float dist_diff = fabs(camAngles[2] - cam_idealdist->value);
        if (dist_diff < 1.5f)
        {
            camAngles[2] = cam_idealdist->value;
        }
        else
        {
            float smooth_factor = CAM_SH_SMOOTH_FACTOR;
            if (cam_sh_smooth_movement)
            {
                smooth_factor = CAM_SH_SMOOTH_FACTOR * CAM_ClampFloat(cam_sh_smooth_movement->value, 0.01f, 1.0f);
            }
            
            // Faster distance adjustment for poor crosshair visibility
            if (enhanced_mode && g_camera_state.crosshair_visibility < CAM_SH_VISIBILITY_THRESHOLD)
            {
                smooth_factor *= 2.5f;
            }
            
            camAngles[2] += (cam_idealdist->value - camAngles[2]) * smooth_factor;
        }
    }

    // Final angle validation
    camAngles[PITCH] = CAM_ClampFloat(camAngles[PITCH], -89.0f, 89.0f);
    camAngles[YAW] = CAM_NormalizeAngle(camAngles[YAW]);
    camAngles[2] = CAM_ClampFloat(camAngles[2], min_dist, max_dist);

    // Apply atmospheric effects with crosshair consideration
    if (enhanced_mode)
    {
        float effect_multiplier = 1.0f;
        if (cam_sh_crosshair_priority && cam_sh_crosshair_priority->value > 0.0f)
        {
            effect_multiplier = 1.0f - (cam_sh_crosshair_priority->value * 0.5f);
        }
        
        camAngles[0] += (g_camera_state.shake_offset[0] + g_camera_state.sway_offset[0]) * effect_multiplier;
        camAngles[1] += (g_camera_state.shake_offset[1] + g_camera_state.sway_offset[1]) * effect_multiplier;
        camAngles[2] += (g_camera_state.shake_offset[2] + g_camera_state.sway_offset[2]) * 0.02f;
    }

    // FINAL VALIDATION AND OUTPUT
    if (CAM_IsValidFloat(camAngles[0]) && CAM_IsValidFloat(camAngles[1]) && CAM_IsValidFloat(camAngles[2]))
    {
        cam_ofs[0] = camAngles[0];
        cam_ofs[1] = camAngles[1];
        cam_ofs[2] = CAM_ClampFloat(camAngles[2], min_dist, max_dist);
    }
    else
    {
        // Fallback to optimal crosshair position
        cam_ofs[0] = 0.0f;
        cam_ofs[1] = 0.0f;
        cam_ofs[2] = CAM_SH_OPTIMAL_DISTANCE;
    }

    // Auto-center for improved crosshair alignment
    if (enhanced_mode && cam_sh_auto_center && cam_sh_auto_center->value > 0.0f)
    {
        if (g_camera_state.crosshair_visibility < 0.5f)
        {
            static float auto_center_timer = 0.0f;
            auto_center_timer += cam_frame_time;
            
            if (auto_center_timer > 1.5f)
            {
                float center_strength = CAM_ClampFloat(cam_sh_auto_center->value, 0.0f, 1.0f) * 0.015f;
                cam_ofs[0] *= (1.0f - center_strength);
                cam_ofs[1] *= (1.0f - center_strength);
                
                if (auto_center_timer > 3.0f)
                    auto_center_timer = 0.0f;
            }
        }
    }
}

//-------------------------------------------------- HELPER FUNCTIONS

float MoveToward(float cur, float goal, float maxspeed)
{
    if (!CAM_IsValidFloat(cur) || !CAM_IsValidFloat(goal) || !CAM_IsValidFloat(maxspeed))
        return cur;
        
    maxspeed = CAM_ClampFloat(maxspeed, 0.1f, 20.0f);
    
    if (fabs(cur - goal) < CAM_SH_EPSILON)
        return goal;

    // Enhanced angle wrapping
    if (fabs(cur - goal) > CAM_SH_ANGLE_WRAP_THRESHOLD)
    {
        if (cur < goal)
            cur += 360.0f;
        else
            cur -= 360.0f;
    }

    float diff = goal - cur;
    float move_amount = diff * 0.2f;
    
    if (fabs(move_amount) > maxspeed)
    {
        move_amount = (move_amount > 0) ? maxspeed : -maxspeed;
    }
    
    cur += move_amount;
    
    if (fabs(cur - goal) < 0.3f)
        cur = goal;

    return CAM_NormalizeAngle(cur);
}

//-------------------------------------------------- API FUNCTIONS

void CAM_SetCameraMode(int mode)
{
    g_camera_state.mode = mode;
    
    switch (mode)
    {
        case CAM_MODE_FIRSTPERSON:
            CAM_ToFirstPerson();
            break;
        case CAM_MODE_THIRDPERSON:
            CAM_ToThirdPerson();
            break;
        case CAM_MODE_FIXEDANGLE:
        case CAM_MODE_DRAMATIC:
        case CAM_MODE_SECURITY:
        case CAM_MODE_CINEMATIC:
            cam_thirdperson = 1;
            break;
    }
    
    gEngfuncs.Cvar_SetValue("cam_command", 0);
}

void CAM_SetCameraTensionLevel(float level)
{
    if (cam_sh_tension_level)
    {
        gEngfuncs.Cvar_SetValue("cam_sh_tension_level", CAM_ClampFloat(level, 0.0f, 1.0f));
    }
}

void CAM_SetAtmosphericShake(float intensity, float duration)
{
    intensity = CAM_ClampFloat(intensity, 0.0f, 15.0f);
    duration = CAM_ClampFloat(duration, 0.0f, 15.0f);
    
    cam_shake_intensity = fmax(cam_shake_intensity, intensity);
    cam_shake_time = fmax(cam_shake_time, duration);
}

void CAM_UpdateColorCorrection(float r, float g, float b)
{
    g_camera_state.color_correction[0] = CAM_ClampFloat(r, 0.0f, 2.0f);
    g_camera_state.color_correction[1] = CAM_ClampFloat(g, 0.0f, 2.0f);
    g_camera_state.color_correction[2] = CAM_ClampFloat(b, 0.0f, 2.0f);
}

void CAM_SetVignetteIntensity(float intensity)
{
    if (cam_sh_vignette)
    {
        gEngfuncs.Cvar_SetValue("cam_sh_vignette", CAM_ClampFloat(intensity, 0.0f, 1.0f));
    }
}

//-------------------------------------------------- ORIGINAL COMPATIBILITY FUNCTIONS

extern void KeyDown(kbutton_t *b);
extern void KeyUp(kbutton_t *b);

void CAM_PitchUpDown(void) { KeyDown(&cam_pitchup); }
void CAM_PitchUpUp(void) { KeyUp(&cam_pitchup); }
void CAM_PitchDownDown(void) { KeyDown(&cam_pitchdown); }
void CAM_PitchDownUp(void) { KeyUp(&cam_pitchdown); }
void CAM_YawLeftDown(void) { KeyDown(&cam_yawleft); }
void CAM_YawLeftUp(void) { KeyUp(&cam_yawleft); }
void CAM_YawRightDown(void) { KeyDown(&cam_yawright); }
void CAM_YawRightUp(void) { KeyUp(&cam_yawright); }
void CAM_InDown(void) { KeyDown(&cam_in); }
void CAM_InUp(void) { KeyUp(&cam_in); }
void CAM_OutDown(void) { KeyDown(&cam_out); }
void CAM_OutUp(void) { KeyUp(&cam_out); }

void CAM_ToThirdPerson(void)
{
    vec3_t viewangles;
#if !_DEBUG
    if (gEngfuncs.GetMaxClients() > 1)
    {
        return;
    }
#endif
    gEngfuncs.GetViewAngles((float *)viewangles);

    if (!cam_thirdperson)
    {
        cam_thirdperson = 1; 
        cam_ofs[YAW] = CAM_IsValidFloat(viewangles[YAW]) ? viewangles[YAW] : 0.0f;
        cam_ofs[PITCH] = CAM_IsValidFloat(viewangles[PITCH]) ? viewangles[PITCH] : 0.0f;
        
        // Set optimal distance for superior crosshair visibility
        bool enhanced = (cam_sh_enable && cam_sh_enable->value);
        cam_ofs[2] = enhanced ? CAM_SH_OPTIMAL_DISTANCE : CAM_MIN_DIST;
        
        if (enhanced && g_camera_state.mode == CAM_MODE_FIRSTPERSON)
        {
            g_camera_state.mode = CAM_MODE_THIRDPERSON;
        }
    }

    gEngfuncs.Cvar_SetValue("cam_command", 0);
}

void CAM_ToFirstPerson(void) 
{ 
    cam_thirdperson = 0;
    g_camera_state.mode = CAM_MODE_FIRSTPERSON;
    gEngfuncs.Cvar_SetValue("cam_command", 0);
}

void CAM_ToggleSnapto(void) 
{ 
    cam_snapto->value = !cam_snapto->value;
}

void CAM_InitSurvivalHorror(void)
{
    memset(&g_camera_state, 0, sizeof(camera_state_t));
    g_camera_state.mode = CAM_MODE_FIRSTPERSON;
    g_camera_state.current_zone = -1;
    g_camera_state.previous_zone = -1;
    g_camera_state.active_sequence = -1;
    g_camera_state.collision_enabled = true;
    g_camera_state.collision_radius = CAM_SH_COLLISION_RADIUS;
    g_camera_state.look_ahead_distance = 48.0f;
    g_camera_state.vignette_intensity = 0.0f;
    g_camera_state.color_correction[0] = 1.0f;
    g_camera_state.color_correction[1] = 1.0f;
    g_camera_state.color_correction[2] = 1.0f;
    g_camera_state.crosshair_visibility = 1.0f;
    g_camera_state.crosshair_obstructed = false;
    
    VectorClear(g_camera_state.crosshair_world_pos);
    VectorClear(g_camera_state.crosshair_screen_pos);
    VectorClear(g_camera_state.aim_target);
}

void CAM_Init(void)
{
    // Register original commands
    gEngfuncs.pfnAddCommand("+campitchup", CAM_PitchUpDown);
    gEngfuncs.pfnAddCommand("-campitchup", CAM_PitchUpUp);
    gEngfuncs.pfnAddCommand("+campitchdown", CAM_PitchDownDown);
    gEngfuncs.pfnAddCommand("-campitchdown", CAM_PitchDownUp);
    gEngfuncs.pfnAddCommand("+camyawleft", CAM_YawLeftDown);
    gEngfuncs.pfnAddCommand("-camyawleft", CAM_YawLeftUp);
    gEngfuncs.pfnAddCommand("+camyawright", CAM_YawRightDown);
    gEngfuncs.pfnAddCommand("-camyawright", CAM_YawRightUp);
    gEngfuncs.pfnAddCommand("+camin", CAM_InDown);
    gEngfuncs.pfnAddCommand("-camin", CAM_InUp);
    gEngfuncs.pfnAddCommand("+camout", CAM_OutDown);
    gEngfuncs.pfnAddCommand("-camout", CAM_OutUp);
    gEngfuncs.pfnAddCommand("thirdperson", CAM_ToThirdPerson);
    gEngfuncs.pfnAddCommand("firstperson", CAM_ToFirstPerson);
    gEngfuncs.pfnAddCommand("+cammousemove", CAM_StartMouseMove);
    gEngfuncs.pfnAddCommand("-cammousemove", CAM_EndMouseMove);
    gEngfuncs.pfnAddCommand("+camdistance", CAM_StartDistance);
    gEngfuncs.pfnAddCommand("-camdistance", CAM_EndDistance);
    gEngfuncs.pfnAddCommand("snapto", CAM_ToggleSnapto);

    // Register original CVars with optimized defaults
    cam_command = gEngfuncs.pfnRegisterVariable("cam_command", "0", 0);
    cam_snapto = gEngfuncs.pfnRegisterVariable("cam_snapto", "0", 0);
    cam_idealyaw = gEngfuncs.pfnRegisterVariable("cam_idealyaw", "0", 0);
    cam_idealpitch = gEngfuncs.pfnRegisterVariable("cam_idealpitch", "0", 0);
    cam_idealdist = gEngfuncs.pfnRegisterVariable("cam_idealdist", "80", 0); // Optimal default
    cam_contain = gEngfuncs.pfnRegisterVariable("cam_contain", "0", 0);
    c_maxpitch = gEngfuncs.pfnRegisterVariable("c_maxpitch", "90.0", 0);
    c_minpitch = gEngfuncs.pfnRegisterVariable("c_minpitch", "0.0", 0);
    c_maxyaw = gEngfuncs.pfnRegisterVariable("c_maxyaw", "135.0", 0);
    c_minyaw = gEngfuncs.pfnRegisterVariable("c_minyaw", "-135.0", 0);
    c_maxdistance = gEngfuncs.pfnRegisterVariable("c_maxdistance", "180.0", 0);
    c_mindistance = gEngfuncs.pfnRegisterVariable("c_mindistance", "24.0", 0);

    // Register enhanced survival horror CVars
    cam_sh_enable = gEngfuncs.pfnRegisterVariable("cam_sh_enable", "1", FCVAR_ARCHIVE);
    cam_sh_mode = gEngfuncs.pfnRegisterVariable("cam_sh_mode", "1", FCVAR_ARCHIVE);
    cam_sh_transition_speed = gEngfuncs.pfnRegisterVariable("cam_sh_transition_speed", "1.0", FCVAR_ARCHIVE);
    cam_sh_atmospheric_shake = gEngfuncs.pfnRegisterVariable("cam_sh_atmospheric_shake", "0.1", FCVAR_ARCHIVE);
    cam_sh_tension_level = gEngfuncs.pfnRegisterVariable("cam_sh_tension_level", "0.0", 0);
    cam_sh_smooth_movement = gEngfuncs.pfnRegisterVariable("cam_sh_smooth_movement", "1.0", FCVAR_ARCHIVE);
    cam_sh_collision_detection = gEngfuncs.pfnRegisterVariable("cam_sh_collision_detection", "1", FCVAR_ARCHIVE);
    cam_sh_dramatic_angles = gEngfuncs.pfnRegisterVariable("cam_sh_dramatic_angles", "1", FCVAR_ARCHIVE);
    cam_sh_vignette = gEngfuncs.pfnRegisterVariable("cam_sh_vignette", "0.0", FCVAR_ARCHIVE);
    cam_sh_color_correction = gEngfuncs.pfnRegisterVariable("cam_sh_color_correction", "1", FCVAR_ARCHIVE);
    cam_sh_audio_reactive = gEngfuncs.pfnRegisterVariable("cam_sh_audio_reactive", "0", FCVAR_ARCHIVE);
    cam_sh_auto_zones = gEngfuncs.pfnRegisterVariable("cam_sh_auto_zones", "1", FCVAR_ARCHIVE);
    cam_sh_cinematic_fov = gEngfuncs.pfnRegisterVariable("cam_sh_cinematic_fov", "75", FCVAR_ARCHIVE);
    cam_sh_security_mode = gEngfuncs.pfnRegisterVariable("cam_sh_security_mode", "0", FCVAR_ARCHIVE);
    cam_sh_follow_speed = gEngfuncs.pfnRegisterVariable("cam_sh_follow_speed", "0.5", FCVAR_ARCHIVE);
    cam_sh_look_ahead = gEngfuncs.pfnRegisterVariable("cam_sh_look_ahead", "0.3", FCVAR_ARCHIVE);
    cam_sh_height_offset = gEngfuncs.pfnRegisterVariable("cam_sh_height_offset", "1.0", FCVAR_ARCHIVE);
    cam_sh_distance_scale = gEngfuncs.pfnRegisterVariable("cam_sh_distance_scale", "1.0", FCVAR_ARCHIVE);
    cam_sh_angle_constraints = gEngfuncs.pfnRegisterVariable("cam_sh_angle_constraints", "1", FCVAR_ARCHIVE);

    // Register crosshair enhancement CVars
    cam_sh_crosshair_priority = gEngfuncs.pfnRegisterVariable("cam_sh_crosshair_priority", "1.0", FCVAR_ARCHIVE);
    cam_sh_shoulder_offset = gEngfuncs.pfnRegisterVariable("cam_sh_shoulder_offset", "1.0", FCVAR_ARCHIVE);
    cam_sh_aim_assist = gEngfuncs.pfnRegisterVariable("cam_sh_aim_assist", "0.2", FCVAR_ARCHIVE);
    cam_sh_auto_center = gEngfuncs.pfnRegisterVariable("cam_sh_auto_center", "0.3", FCVAR_ARCHIVE);

    // Initialize enhanced system
    CAM_InitSurvivalHorror();

	CAM_UpdateCVARs();
}

void CAM_ClearStates(void)
{
    vec3_t viewangles;
    gEngfuncs.GetViewAngles((float *)viewangles);

    // Clear button states
    cam_pitchup.state = 0;
    cam_pitchdown.state = 0;
    cam_yawleft.state = 0;
    cam_yawright.state = 0;
    cam_in.state = 0;
    cam_out.state = 0;

    // Clear camera state
    cam_thirdperson = 0;
    cam_command->value = 0;
    cam_mousemove = 0;
    cam_snapto->value = 0;
    cam_distancemove = 0;

    cam_ofs[0] = 0.0f;
    cam_ofs[1] = 0.0f;
    
    bool enhanced = (cam_sh_enable && cam_sh_enable->value);
    cam_ofs[2] = enhanced ? CAM_SH_OPTIMAL_DISTANCE : CAM_MIN_DIST;

    if (CAM_IsValidVector(viewangles))
    {
        cam_idealpitch->value = viewangles[PITCH];
        cam_idealyaw->value = viewangles[YAW];
    }
    else
    {
        cam_idealpitch->value = 0.0f;
        cam_idealyaw->value = 0.0f;
    }
    cam_idealdist->value = enhanced ? CAM_SH_OPTIMAL_DISTANCE : CAM_MIN_DIST;

    // Clear enhanced states
    if (enhanced)
    {
        g_camera_state.mode = CAM_MODE_FIRSTPERSON;
        g_camera_state.current_zone = -1;
        g_camera_state.previous_zone = -1;
        g_camera_state.transition.in_transition = false;
        g_camera_state.atmospheric_shake = 0.0f;
        g_camera_state.tension_level = 0.0f;
        g_camera_state.crosshair_visibility = 1.0f;
        g_camera_state.crosshair_obstructed = false;
        
        VectorClear(g_camera_state.shake_offset);
        VectorClear(g_camera_state.sway_offset);
        VectorClear(g_camera_state.player_velocity);
        VectorClear(g_camera_state.crosshair_world_pos);
        VectorClear(g_camera_state.aim_target);
        
        cam_shake_time = 0.0f;
        cam_shake_intensity = 0.0f;
        VectorClear(cam_atmospheric_offset);
        cam_sway_time = 0.0f;
        cam_last_zone = -1;
        cam_zone_transition_time = 0.0f;
    }
}

void CAM_StartMouseMove(void)
{
    float flSensitivity;

    if (g_iUser1 != 0)
    {
        cam_mousemove = 0;
        iMouseInUse = 0;
        return;
    }

    if (cam_thirdperson || g_camera_state.mode != CAM_MODE_FIRSTPERSON)
    {
        if (!cam_mousemove)
        {
            cam_mousemove = 1;
            iMouseInUse = 1;
            GetCursorPos(&cam_mouse);

            if ((flSensitivity = gHUD.GetSensitivity()) != 0)
            {
                cam_old_mouse_x = cam_mouse.x * flSensitivity;
                cam_old_mouse_y = cam_mouse.y * flSensitivity;
            }
            else
            {
                cam_old_mouse_x = cam_mouse.x;
                cam_old_mouse_y = cam_mouse.y;
            }
        }
    }
    else
    {   
        cam_mousemove = 0;
        iMouseInUse = 0;
    }
}

void CAM_EndMouseMove(void)
{
    cam_mousemove = 0;
    iMouseInUse = 0;
}

void CAM_StartDistance(void)
{
    if (cam_thirdperson || g_camera_state.mode != CAM_MODE_FIRSTPERSON)
    {
        if (!cam_distancemove)
        {
            cam_distancemove = 1;
            cam_mousemove = 1;
            iMouseInUse = 1;
            GetCursorPos(&cam_mouse);
            cam_old_mouse_x = cam_mouse.x * gHUD.GetSensitivity();
            cam_old_mouse_y = cam_mouse.y * gHUD.GetSensitivity();
        }
    }
    else
    {
        cam_distancemove = 0;
        cam_mousemove = 0;
        iMouseInUse = 0;
    }
}

void CAM_EndDistance(void)
{
    cam_distancemove = 0;
    cam_mousemove = 0;
    iMouseInUse = 0;
}

int DLLEXPORT CL_IsThirdPerson(void)
{
    return (cam_thirdperson ? 1 : 0) || 
           (g_camera_state.mode != CAM_MODE_FIRSTPERSON) ||
           (g_iUser1 && (g_iUser2 == gEngfuncs.GetLocalPlayer()->index));
}

void DLLEXPORT CL_CameraOffset(float *ofs)
{
    if (ofs && CAM_IsValidVector(cam_ofs))
    {
        VectorCopy(cam_ofs, ofs);
    }
    else if (ofs)
    {
        VectorClear(ofs);
    }
}

void CAM_UpdateCVARs(void)
{
    // Always get the latest pointers to ensure they are not stale.
    cam_sh_enable = gEngfuncs.pfnGetCvarPointer("cam_sh_enable");
    cam_sh_mode = gEngfuncs.pfnGetCvarPointer("cam_sh_mode");
    cam_sh_transition_speed = gEngfuncs.pfnGetCvarPointer("cam_sh_transition_speed");
    cam_sh_atmospheric_shake = gEngfuncs.pfnGetCvarPointer("cam_sh_atmospheric_shake");
    cam_sh_tension_level = gEngfuncs.pfnGetCvarPointer("cam_sh_tension_level");
    cam_sh_smooth_movement = gEngfuncs.pfnGetCvarPointer("cam_sh_smooth_movement");
    cam_sh_collision_detection = gEngfuncs.pfnGetCvarPointer("cam_sh_collision_detection");
    cam_sh_dramatic_angles = gEngfuncs.pfnGetCvarPointer("cam_sh_dramatic_angles");
    cam_sh_vignette = gEngfuncs.pfnGetCvarPointer("cam_sh_vignette");
    cam_sh_color_correction = gEngfuncs.pfnGetCvarPointer("cam_sh_color_correction");
    cam_sh_audio_reactive = gEngfuncs.pfnGetCvarPointer("cam_sh_audio_reactive");
    cam_sh_auto_zones = gEngfuncs.pfnGetCvarPointer("cam_sh_auto_zones");
    cam_sh_cinematic_fov = gEngfuncs.pfnGetCvarPointer("cam_sh_cinematic_fov");
    cam_sh_security_mode = gEngfuncs.pfnGetCvarPointer("cam_sh_security_mode");
    cam_sh_follow_speed = gEngfuncs.pfnGetCvarPointer("cam_sh_follow_speed");
    cam_sh_look_ahead = gEngfuncs.pfnGetCvarPointer("cam_sh_look_ahead");
    cam_sh_height_offset = gEngfuncs.pfnGetCvarPointer("cam_sh_height_offset");
    cam_sh_distance_scale = gEngfuncs.pfnGetCvarPointer("cam_sh_distance_scale");
    cam_sh_angle_constraints = gEngfuncs.pfnGetCvarPointer("cam_sh_angle_constraints");
    cam_sh_crosshair_priority = gEngfuncs.pfnGetCvarPointer("cam_sh_crosshair_priority");
    cam_sh_shoulder_offset = gEngfuncs.pfnGetCvarPointer("cam_sh_shoulder_offset");
    cam_sh_aim_assist = gEngfuncs.pfnGetCvarPointer("cam_sh_aim_assist");
    cam_sh_auto_center = gEngfuncs.pfnGetCvarPointer("cam_sh_auto_center");
}

// Stub functions for incomplete features
void CAM_CalculateFixedAngleCamera(void) {}
void CAM_UpdateCinematicSequence(void) {}
void CAM_CalculateDramaticCamera(void) {}
void CAM_CalculateSecurityCamera(void) {}