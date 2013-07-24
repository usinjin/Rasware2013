#ifndef __DEADRECKONING_H__
#define __DEADRECKONING_H__

typedef struct {
    float x; // units (as defined by arguments to initDeadReckoning)
    float y; // units
    float heading; // radians
    float v; // units/second
    float w; // radians/second
} tPose;

// copies _pose into the current pose
void SetCurrentPose(tPose *_pose);

// copies the internal pose into _pose
void GetCurrentPose(tPose *_pose);

// This starts differential dead reckoning using a left and right encoder and a periodic timer event.
// Every timeStep seconds, an internal pose is updated with an estimation of the position (in units) and direction (in radians) of the center of the hypothetical wheel axis. 
// Note: This assumes that the left and right encoders have been initialized
void InitDeadReckoning(
    tPose *initialPose,
    float unitsAxisWidth, // where 'units' could be inches, meters, etc.
    float ticksPerUnit, // units must be consistent with axis width
    float timeStep, // seconds
    unsigned char leftEnc,
    unsigned char rightEnc
    );

#endif // __DEADRECKONING_H__
