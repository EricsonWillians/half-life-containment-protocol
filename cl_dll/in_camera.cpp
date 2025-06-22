//========= Copyright (c) 1996-2002, Valve LLC, All rights reserved. ============
//
// Purpose: Professional Over-The-Shoulder Camera System
//          Optimized for Horror Action Shooters - Resident Evil Style
//          Complete rewrite for superior crosshair alignment and combat precision
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

//================================================
// PROFESSIONAL CAMERA CONSTANTS
//================================================

// Over-the-shoulder positioning - Optimized for crosshair visibility
#define CAM_SHOULDER_DISTANCE       75.0f   // Perfect tactical distance
#define CAM_SHOULDER_RIGHT_OFFSET   22.0f   // Right shoulder offset
#define CAM_SHOULDER_UP_OFFSET      8.0f    // Height offset above center
#define CAM_SHOULDER_FORWARD_BIAS   -4.0f   // Slight forward bias for better view

// Distance constraints
#define CAM_MIN_DISTANCE           20.0f
#define CAM_MAX_DISTANCE           150.0f
#define CAM_COLLISION_BUFFER       12.0f

// Smoothing and responsiveness
#define CAM_SMOOTH_FACTOR          0.12f    // Camera position smoothing
#define CAM_ANGLE_SMOOTH           0.18f    // Angle smoothing
#define CAM_COLLISION_SMOOTH       0.25f    // Collision response speed

// Mouse sensitivity multipliers
#define CAM_MOUSE_YAW_SCALE        0.022f   // Horizontal sensitivity
#define CAM_MOUSE_PITCH_SCALE      0.018f   // Vertical sensitivity (slightly reduced)

// Angle constraints for over-the-shoulder
#define CAM_MAX_PITCH              85.0f
#define CAM_MIN_PITCH              -85.0f
#define CAM_MAX_YAW                180.0f
#define CAM_MIN_YAW                -180.0f

// Safety and validation
#define CAM_EPSILON                0.001f
#define CAM_MAX_TRACE_DISTANCE     2048.0f

//================================================
// GLOBAL VARIABLES
//================================================

// Core camera state
camera_state_t g_camera_state;
vec3_t cam_ofs;
int cam_thirdperson;

// Input handling
int cam_mousemove = 0;
int iMouseInUse = 0;
int cam_distancemove = 0;
POINT cam_mouse;
int cam_old_mouse_x, cam_old_mouse_y;

// CVars
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

// Enhanced CVars
cvar_t *cam_shoulder_distance;
cvar_t *cam_shoulder_right;
cvar_t *cam_shoulder_up;
cvar_t *cam_collision_enabled;
cvar_t *cam_smooth_movement;
cvar_t *cam_mouse_sensitivity;

// Input buttons
static kbutton_t cam_pitchup, cam_pitchdown, cam_yawleft, cam_yawright;
static kbutton_t cam_in, cam_out, cam_move;

// Frame timing
float cam_last_time = 0.0f;
float cam_frametime = 0.0f;

//================================================
// UTILITY FUNCTIONS
//================================================

bool CAM_IsValidFloat(float f)
{
    return !isnan(f) && isfinite(f) && f > -1e6f && f < 1e6f;
}

bool CAM_IsValidVector(vec3_t v)
{
    return v && CAM_IsValidFloat(v[0]) && CAM_IsValidFloat(v[1]) && CAM_IsValidFloat(v[2]);
}

float CAM_ClampFloat(float value, float min_val, float max_val)
{
    if (!CAM_IsValidFloat(value)) return min_val;
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

float CAM_NormalizeAngle(float angle)
{
    if (!CAM_IsValidFloat(angle)) return 0.0f;
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

float CAM_AngleDifference(float a1, float a2)
{
    float diff = a2 - a1;
    return CAM_NormalizeAngle(diff);
}

float CAM_GetCvarFloat(cvar_t *cvar, float default_value)
{
    if (cvar && CAM_IsValidFloat(cvar->value))
        return cvar->value;
    return default_value;
}

float VectorLength(vec3_t v)
{
    if (!CAM_IsValidVector(v)) return 0.0f;
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

//================================================
// COLLISION DETECTION
//================================================

bool CAM_TraceCollision(vec3_t start, vec3_t end, vec3_t result)
{
    if (!CAM_IsValidVector(start) || !CAM_IsValidVector(end) || !result)
    {
        if (result) VectorCopy(end, result);
        return false;
    }

    pmtrace_t trace;
    memset(&trace, 0, sizeof(trace));

    // Validate distance
    vec3_t trace_dir;
    VectorSubtract(end, start, trace_dir);
    float trace_dist = VectorLength(trace_dir);
    
    if (trace_dist < CAM_EPSILON)
    {
        VectorCopy(start, result);
        return false;
    }

    if (trace_dist > CAM_MAX_TRACE_DISTANCE)
    {
        VectorNormalize(trace_dir);
        VectorMA(start, CAM_MAX_TRACE_DISTANCE, trace_dir, end);
    }

    // Perform trace
    if (gEngfuncs.pEventAPI && gEngfuncs.pEventAPI->EV_SetUpPlayerPrediction)
    {
        gEngfuncs.pEventAPI->EV_SetUpPlayerPrediction(false, true);
        
        if (gEngfuncs.pEventAPI->EV_PushPMStates)
            gEngfuncs.pEventAPI->EV_PushPMStates();
        
        cl_entity_t *local = gEngfuncs.GetLocalPlayer();
        int player_idx = local ? local->index : -1;
        
        if (gEngfuncs.pEventAPI->EV_SetSolidPlayers)
            gEngfuncs.pEventAPI->EV_SetSolidPlayers(player_idx);
        
        if (gEngfuncs.pEventAPI->EV_SetTraceHull)
            gEngfuncs.pEventAPI->EV_SetTraceHull(2);
        
        if (gEngfuncs.pEventAPI->EV_PlayerTrace)
        {
            gEngfuncs.pEventAPI->EV_PlayerTrace(start, end, PM_GLASS_IGNORE | PM_STUDIO_BOX, player_idx, &trace);
        }
        
        if (gEngfuncs.pEventAPI->EV_PopPMStates)
            gEngfuncs.pEventAPI->EV_PopPMStates();
    }

    // Process results
    if (trace.fraction < 1.0f && trace.fraction > 0.0f)
    {
        vec3_t safe_pos;
        VectorNormalize(trace_dir);
        VectorMA(trace.endpos, -CAM_COLLISION_BUFFER, trace_dir, safe_pos);
        VectorCopy(safe_pos, result);
        return true;
    }
    else
    {
        VectorCopy(end, result);
        return false;
    }
}

//================================================
// FIXED OVER-THE-SHOULDER CAMERA CALCULATION
//================================================

void CAM_CalculateShoulderPosition(vec3_t player_origin, vec3_t view_angles, 
                                  float distance, vec3_t *cam_origin, vec3_t *cam_angles)
{
    if (!CAM_IsValidVector(player_origin) || !CAM_IsValidVector(view_angles) || 
        !cam_origin || !cam_angles)
        return;

    // Get customizable offsets
    float shoulder_distance = CAM_GetCvarFloat(cam_shoulder_distance, CAM_SHOULDER_DISTANCE);
    float shoulder_right = CAM_GetCvarFloat(cam_shoulder_right, CAM_SHOULDER_RIGHT_OFFSET);
    float shoulder_up = CAM_GetCvarFloat(cam_shoulder_up, CAM_SHOULDER_UP_OFFSET);

    // CRITICAL FIX: Use the actual camera angles, not just view angles
    vec3_t cam_view_angles;
    cam_view_angles[PITCH] = cam_ofs[PITCH];
    cam_view_angles[YAW] = cam_ofs[YAW];
    cam_view_angles[ROLL] = 0.0f;

    // Calculate direction vectors from CAMERA angles (not player view angles)
    vec3_t forward, right, up;
    AngleVectors(cam_view_angles, forward, right, up);

    if (!CAM_IsValidVector(forward) || !CAM_IsValidVector(right) || !CAM_IsValidVector(up))
        return;

    // Start from player eye position
    vec3_t eye_origin;
    VectorCopy(player_origin, eye_origin);
    eye_origin[2] += 28.0f; // Standard eye height

    // STEP 1: Calculate base camera position behind player
    vec3_t base_pos;
    VectorMA(eye_origin, -shoulder_distance, forward, base_pos);

    // STEP 2: Apply RIGHT shoulder offset (this creates the over-shoulder effect)
    vec3_t shoulder_pos;
    VectorMA(base_pos, shoulder_right, right, shoulder_pos);
    
    // STEP 3: Apply vertical offset
    vec3_t final_ideal_pos;
    VectorMA(shoulder_pos, shoulder_up, up, final_ideal_pos);

    // STEP 4: Apply slight forward bias for better view
    VectorMA(final_ideal_pos, CAM_SHOULDER_FORWARD_BIAS, forward, final_ideal_pos);

    // Check for collisions
    vec3_t final_pos;
    bool collision_enabled = cam_collision_enabled ? (cam_collision_enabled->value != 0.0f) : true;
    
    if (collision_enabled)
    {
        if (CAM_TraceCollision(eye_origin, final_ideal_pos, final_pos))
        {
            // Collision detected, but maintain shoulder offset
            vec3_t collision_dir;
            VectorSubtract(final_pos, eye_origin, collision_dir);
            float collision_dist = VectorLength(collision_dir);
            
            if (collision_dist > CAM_MIN_DISTANCE)
            {
                // Maintain the shoulder offset even when colliding
                VectorNormalize(collision_dir);
                vec3_t safe_back_pos;
                VectorMA(eye_origin, collision_dist - CAM_COLLISION_BUFFER, collision_dir, safe_back_pos);
                
                // Re-apply shoulder offset proportionally
                float offset_scale = (collision_dist - CAM_COLLISION_BUFFER) / shoulder_distance;
                offset_scale = CAM_ClampFloat(offset_scale, 0.3f, 1.0f); // Minimum 30% offset
                
                VectorMA(safe_back_pos, shoulder_right * offset_scale, right, *cam_origin);
                (*cam_origin)[2] += shoulder_up * offset_scale;
            }
            else
            {
                // Use minimum safe position with reduced offset
                VectorNormalize(collision_dir);
                VectorMA(eye_origin, CAM_MIN_DISTANCE, collision_dir, *cam_origin);
                VectorMA(*cam_origin, shoulder_right * 0.5f, right, *cam_origin);
            }
        }
        else
        {
            VectorCopy(final_ideal_pos, *cam_origin);
        }
    }
    else
    {
        VectorCopy(final_ideal_pos, *cam_origin);
    }

    // Set camera angles to look toward crosshair target
    VectorCopy(cam_view_angles, *cam_angles);
    
    // Calculate crosshair world position for perfect aiming
    vec3_t crosshair_target;
    VectorMA(eye_origin, 8192.0f, forward, crosshair_target);
    
    // Store crosshair info for HUD
    VectorCopy(crosshair_target, g_camera_state.crosshair_world_pos);
    g_camera_state.crosshair_visibility = 1.0f;
}

//================================================
// FIXED MAIN CAMERA THINK FUNCTION
//================================================

void DLLEXPORT CAM_Think(void)
{
    vec3_t viewangles, player_origin;
    float flSensitivity;

    // Update frame timing
    float current_time = gEngfuncs.GetClientTime();
    cam_frametime = current_time - cam_last_time;
    cam_last_time = current_time;
    
    if (!CAM_IsValidFloat(cam_frametime) || cam_frametime <= 0.0f)
        cam_frametime = 0.016f;
    
    cam_frametime = CAM_ClampFloat(cam_frametime, 0.001f, 0.1f);

    // Get player information
    cl_entity_t *local_player = gEngfuncs.GetLocalPlayer();
    if (!local_player)
        return;

    VectorCopy(local_player->origin, player_origin);
    gEngfuncs.GetViewAngles(viewangles);

    if (!CAM_IsValidVector(player_origin) || !CAM_IsValidVector(viewangles))
        return;

    // Disable third person in multiplayer
    if (gEngfuncs.GetMaxClients() > 1 && cam_thirdperson)
    {
        CAM_ToFirstPerson();
        return;
    }

    // Handle camera commands
    int cam_cmd = cam_command ? (int)cam_command->value : 0;
    switch (cam_cmd)
    {
        case 1:
            CAM_ToThirdPerson();
            break;
        case 2:
            CAM_ToFirstPerson();
            break;
    }

    // Early exit for first person
    if (!cam_thirdperson)
        return;

    // CRITICAL FIX: Initialize camera angles from PLAYER view angles
    if (cam_ofs[0] == 0.0f && cam_ofs[1] == 0.0f && cam_ofs[2] == 0.0f)
    {
        cam_ofs[PITCH] = viewangles[PITCH];
        cam_ofs[YAW] = viewangles[YAW];
        cam_ofs[2] = CAM_GetCvarFloat(cam_shoulder_distance, CAM_SHOULDER_DISTANCE);
    }

    // Current camera state
    vec3_t cam_angles;
    cam_angles[PITCH] = cam_ofs[PITCH];
    cam_angles[YAW] = cam_ofs[YAW];
    cam_angles[ROLL] = 0.0f;
    
    float distance = cam_ofs[2];
    distance = CAM_ClampFloat(distance, CAM_MIN_DISTANCE, CAM_MAX_DISTANCE);

    //================================================
    // MOUSE CONTROL - CRITICAL FIX
    //================================================

    if (cam_mousemove && !cam_distancemove)
    {
        POINT current_mouse;
        GetCursorPos(&current_mouse);

        int center_x = gEngfuncs.GetWindowCenterX();
        int center_y = gEngfuncs.GetWindowCenterY();
        
        int mouse_delta_x = current_mouse.x - center_x;
        int mouse_delta_y = current_mouse.y - center_y;
        
        if (abs(mouse_delta_x) > 0 || abs(mouse_delta_y) > 0)
        {
            flSensitivity = gHUD.GetSensitivity();
            if (!CAM_IsValidFloat(flSensitivity) || flSensitivity <= 0.0f)
                flSensitivity = 1.0f;

            float mouse_scale = CAM_GetCvarFloat(cam_mouse_sensitivity, 1.0f);
            
            // Calculate mouse movement and apply to CAMERA angles
            float yaw_delta = (float)mouse_delta_x * CAM_MOUSE_YAW_SCALE * flSensitivity * mouse_scale;
            float pitch_delta = (float)mouse_delta_y * CAM_MOUSE_PITCH_SCALE * flSensitivity * mouse_scale;
            
            yaw_delta = CAM_ClampFloat(yaw_delta, -15.0f, 15.0f);
            pitch_delta = CAM_ClampFloat(pitch_delta, -12.0f, 12.0f);
            
            // Apply to camera angles
            cam_angles[YAW] -= yaw_delta;
            cam_angles[PITCH] += pitch_delta;
            
            // Normalize and constrain
            cam_angles[YAW] = CAM_NormalizeAngle(cam_angles[YAW]);
            cam_angles[PITCH] = CAM_ClampFloat(cam_angles[PITCH], CAM_MIN_PITCH, CAM_MAX_PITCH);
            
            SetCursorPos(center_x, center_y);
        }
    }
    else if (cam_mousemove && cam_distancemove)
    {
        // Distance control
        POINT current_mouse;
        GetCursorPos(&current_mouse);
        
        int center_y = gEngfuncs.GetWindowCenterY();
        int mouse_delta_y = current_mouse.y - center_y;
        
        if (abs(mouse_delta_y) > 0)
        {
            float dist_delta = (float)mouse_delta_y * 0.5f;
            distance += dist_delta;
            distance = CAM_ClampFloat(distance, CAM_MIN_DISTANCE, CAM_MAX_DISTANCE);
            
            SetCursorPos(gEngfuncs.GetWindowCenterX(), center_y);
        }
    }

    // Keyboard controls
    float angle_speed = 90.0f * cam_frametime;
    float dist_speed = 60.0f * cam_frametime;
    
    if (CL_KeyState(&cam_pitchup))
        cam_angles[PITCH] += angle_speed;
    else if (CL_KeyState(&cam_pitchdown))
        cam_angles[PITCH] -= angle_speed;

    if (CL_KeyState(&cam_yawleft))
        cam_angles[YAW] += angle_speed;
    else if (CL_KeyState(&cam_yawright))
        cam_angles[YAW] -= angle_speed;

    if (CL_KeyState(&cam_in))
        distance -= dist_speed;
    else if (CL_KeyState(&cam_out))
        distance += dist_speed;

    // Final constraints
    cam_angles[PITCH] = CAM_ClampFloat(cam_angles[PITCH], CAM_MIN_PITCH, CAM_MAX_PITCH);
    cam_angles[YAW] = CAM_NormalizeAngle(cam_angles[YAW]);
    distance = CAM_ClampFloat(distance, CAM_MIN_DISTANCE, CAM_MAX_DISTANCE);

    //================================================
    // SMOOTH CAMERA MOVEMENT
    //================================================

    bool snap_to = cam_snapto ? (cam_snapto->value != 0.0f) : false;
    
    if (!snap_to)
    {
        float smooth_factor = CAM_GetCvarFloat(cam_smooth_movement, 1.0f);
        float angle_smooth = CAM_ANGLE_SMOOTH * smooth_factor;
        float pos_smooth = CAM_SMOOTH_FACTOR * smooth_factor;
        
        // Smooth angles
        float pitch_diff = cam_angles[PITCH] - cam_ofs[PITCH];
        float yaw_diff = CAM_AngleDifference(cam_ofs[YAW], cam_angles[YAW]);
        
        cam_ofs[PITCH] += pitch_diff * angle_smooth;
        cam_ofs[YAW] += yaw_diff * angle_smooth;
        cam_ofs[YAW] = CAM_NormalizeAngle(cam_ofs[YAW]);
        
        // Smooth distance
        cam_ofs[2] += (distance - cam_ofs[2]) * pos_smooth;
    }
    else
    {
        cam_ofs[PITCH] = cam_angles[PITCH];
        cam_ofs[YAW] = cam_angles[YAW];
        cam_ofs[2] = distance;
    }

    //================================================
    // CALCULATE FINAL CAMERA POSITION - CRITICAL FIX
    //================================================

    vec3_t final_cam_origin, final_cam_angles;
    CAM_CalculateShoulderPosition(player_origin, viewangles, cam_ofs[2], 
                                 &final_cam_origin, &final_cam_angles);

    // CRITICAL FIX: Don't sync view angles back to camera!
    // The camera should be independent of player view angles
    // Only sync if we need crosshair alignment
    
    // For debugging - you can uncomment this to see where the camera should be
    /*
    vec3_t debug_angles;
    VectorCopy(cam_ofs, debug_angles);
    debug_angles[2] = 0; // Clear distance from angles
    gEngfuncs.SetViewAngles(debug_angles);
    */

    // Update CVars for debugging
    if (cam_idealpitch) cam_idealpitch->value = cam_ofs[PITCH];
    if (cam_idealyaw) cam_idealyaw->value = cam_ofs[YAW];
    if (cam_idealdist) cam_idealdist->value = cam_ofs[2];
    if (cam_command) cam_command->value = 0;
}

//================================================
// COMMAND HANDLERS
//================================================

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
    if (gEngfuncs.GetMaxClients() > 1)
        return;

    if (!gEngfuncs.GetLocalPlayer())
        return;

    if (!cam_thirdperson)
    {
        vec3_t viewangles;
        gEngfuncs.GetViewAngles(viewangles);
        
        cam_thirdperson = 1;
        
        // CRITICAL FIX: Properly initialize camera offset
        cam_ofs[PITCH] = CAM_IsValidFloat(viewangles[PITCH]) ? viewangles[PITCH] : 0.0f;
        cam_ofs[YAW] = CAM_IsValidFloat(viewangles[YAW]) ? viewangles[YAW] : 0.0f;
        cam_ofs[2] = CAM_GetCvarFloat(cam_shoulder_distance, CAM_SHOULDER_DISTANCE);
        
        // Debug print
        gEngfuncs.Con_Printf("Camera activated: Distance=%.1f, Right=%.1f, Up=%.1f\n", 
                           cam_ofs[2], 
                           CAM_GetCvarFloat(cam_shoulder_right, CAM_SHOULDER_RIGHT_OFFSET),
                           CAM_GetCvarFloat(cam_shoulder_up, CAM_SHOULDER_UP_OFFSET));
    }

    if (cam_command) cam_command->value = 0;
}

void CAM_ToFirstPerson(void)
{
    cam_thirdperson = 0;
    if (cam_command) cam_command->value = 0;
}

void CAM_ToggleSnapto(void)
{
    if (cam_snapto)
        cam_snapto->value = !cam_snapto->value;
}

void CAM_StartMouseMove(void)
{
    if (g_iUser1 != 0)
    {
        cam_mousemove = 0;
        iMouseInUse = 0;
        return;
    }

    if (cam_thirdperson)
    {
        if (!cam_mousemove)
        {
            cam_mousemove = 1;
            iMouseInUse = 1;
            GetCursorPos(&cam_mouse);
            
            float sensitivity = gHUD.GetSensitivity();
            if (CAM_IsValidFloat(sensitivity))
            {
                cam_old_mouse_x = cam_mouse.x * sensitivity;
                cam_old_mouse_y = cam_mouse.y * sensitivity;
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
    if (cam_thirdperson)
    {
        if (!cam_distancemove)
        {
            cam_distancemove = 1;
            cam_mousemove = 1;
            iMouseInUse = 1;
            GetCursorPos(&cam_mouse);
            
            float sensitivity = gHUD.GetSensitivity();
            if (CAM_IsValidFloat(sensitivity))
            {
                cam_old_mouse_x = cam_mouse.x * sensitivity;
                cam_old_mouse_y = cam_mouse.y * sensitivity;
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

//================================================
// ENGINE INTERFACE FUNCTIONS
//================================================

int DLLEXPORT CL_IsThirdPerson(void)
{
    return cam_thirdperson || (g_iUser1 && gEngfuncs.GetLocalPlayer() && 
                              (g_iUser2 == gEngfuncs.GetLocalPlayer()->index));
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

//================================================
// INITIALIZATION AND CLEANUP
//================================================

void CAM_Init(void)
{
    // Register commands
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

    // Register CVars
    cam_command = gEngfuncs.pfnRegisterVariable("cam_command", "0", 0);
    cam_snapto = gEngfuncs.pfnRegisterVariable("cam_snapto", "0", 0);
    cam_idealyaw = gEngfuncs.pfnRegisterVariable("cam_idealyaw", "0", 0);
    cam_idealpitch = gEngfuncs.pfnRegisterVariable("cam_idealpitch", "0", 0);
    cam_idealdist = gEngfuncs.pfnRegisterVariable("cam_idealdist", "75", 0);
    cam_contain = gEngfuncs.pfnRegisterVariable("cam_contain", "0", 0);
    c_maxpitch = gEngfuncs.pfnRegisterVariable("c_maxpitch", "85.0", 0);
    c_minpitch = gEngfuncs.pfnRegisterVariable("c_minpitch", "-85.0", 0);
    c_maxyaw = gEngfuncs.pfnRegisterVariable("c_maxyaw", "180.0", 0);
    c_minyaw = gEngfuncs.pfnRegisterVariable("c_minyaw", "-180.0", 0);
    c_maxdistance = gEngfuncs.pfnRegisterVariable("c_maxdistance", "150.0", 0);
    c_mindistance = gEngfuncs.pfnRegisterVariable("c_mindistance", "20.0", 0);

    // Enhanced CVars
    cam_shoulder_distance = gEngfuncs.pfnRegisterVariable("cam_shoulder_distance", "75.0", FCVAR_ARCHIVE);
    cam_shoulder_right = gEngfuncs.pfnRegisterVariable("cam_shoulder_right", "22.0", FCVAR_ARCHIVE);
    cam_shoulder_up = gEngfuncs.pfnRegisterVariable("cam_shoulder_up", "8.0", FCVAR_ARCHIVE);
    cam_collision_enabled = gEngfuncs.pfnRegisterVariable("cam_collision_enabled", "1", FCVAR_ARCHIVE);
    cam_smooth_movement = gEngfuncs.pfnRegisterVariable("cam_smooth_movement", "1.0", FCVAR_ARCHIVE);
    cam_mouse_sensitivity = gEngfuncs.pfnRegisterVariable("cam_mouse_sensitivity", "1.0", FCVAR_ARCHIVE);

    // Initialize state
    memset(&g_camera_state, 0, sizeof(g_camera_state));
    VectorClear(cam_ofs);
    cam_thirdperson = 0;
    cam_mousemove = 0;
    iMouseInUse = 0;
    cam_distancemove = 0;
}

void CAM_ClearStates(void)
{
    // Clear button states
    cam_pitchup.state = 0;
    cam_pitchdown.state = 0;
    cam_yawleft.state = 0;
    cam_yawright.state = 0;
    cam_in.state = 0;
    cam_out.state = 0;

    // Reset camera
    cam_thirdperson = 0;
    cam_mousemove = 0;
    cam_distancemove = 0;
    iMouseInUse = 0;
    
    VectorClear(cam_ofs);
    if (cam_command) cam_command->value = 0;
    if (cam_snapto) cam_snapto->value = 0;

    // Clear enhanced state
    memset(&g_camera_state, 0, sizeof(g_camera_state));
}

//================================================
// STUB FUNCTIONS FOR COMPATIBILITY
//================================================

void CAM_InitSurvivalHorror(void) { CAM_Init(); }
void CAM_UpdateSurvivalHorror(void) {}
void CAM_LoadCameraZones(const char *mapname) {}
void CAM_SetCameraMode(int mode) {}
void CAM_SetTensionLevel(float level) {}
void CAM_TriggerCinematic(int sequence_id) {}
void CAM_AddAtmosphericShake(float intensity, float duration) {}
void CAM_SetColorCorrection(float r, float g, float b) {}
void CAM_EnableVignette(float intensity) {}
void CAM_CalculateFixedAngleCamera(void) {}
void CAM_UpdateCinematicSequence(void) {}
void CAM_CalculateDramaticCamera(void) {}
void CAM_CalculateSecurityCamera(void) {}
void CAM_CalculateCrosshairWorldPosition(vec3_t view_origin, vec3_t view_angles) {}
void CAM_OptimizeCameraForCrosshair(vec3_t player_origin, vec3_t base_cam_pos, vec3_t *final_cam_pos, vec3_t *final_cam_angles) {}

float MoveToward(float cur, float goal, float maxspeed)
{
    if (!CAM_IsValidFloat(cur) || !CAM_IsValidFloat(goal) || !CAM_IsValidFloat(maxspeed))
        return cur;
    
    if (fabs(cur - goal) < CAM_EPSILON)
        return goal;
    
    float diff = goal - cur;
    float step = diff * 0.2f;
    
    if (fabs(step) > maxspeed)
        step = (step > 0) ? maxspeed : -maxspeed;
    
    return cur + step;
}