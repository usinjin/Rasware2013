#ifndef __DEADRECKONING_H__
#define __DEADRECKONING_H__

typedef struct {
    float x; // inches
    float y; // inches
    float heading; // radians
} RobotPose;

// This starts differential dead reckoning using a left and right encoder and a periodic timer event.
// Returns a pointer to a pose that is updated every timeStep containing an estimation of the position (in inches) and direction (in radians) of the center of the hypothetical wheel axis. 
// Note: This assumes that the left and right encoders have been initialized
RobotPose* initDeadReckoning(
    RobotPose *initialPose,
    float inchesAxisWidth,
    float ticksPerInch,
    float timeStep, // seconds
    unsigned char leftEnc,
    unsigned char rightEnc
    );

#endif // __DEADRECKONING_H__
