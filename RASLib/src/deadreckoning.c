#include <math.h>
#include "deadreckoning.h"
#include "encoder.h"
#include "time.h"

static unsigned char isInitialized = 0;

static RobotPose pose;
static float inchesAxisWidth,
             ticksPerInch; // the axis width should be provided in inches, not ticks, so we need this conversion
static unsigned char leftEncIndex,
                     rightEncIndex;

// this bounds the angle t to be within the range [0, 2*PI)
//  where t is can be an angle on the range (-inf, inf)
float boundAngle(float t) {
    float nf = t/(2*M_PI);
    int n = (nf < 0) ? nf - 1 : nf;
    
    return t - n*2*M_PI;
}

// use this to deal with small floating-point errors
int floatBasicallyEqual(float a, float b) {
    return fabs(a - b) < 1.0e-6;
}

// this function will be the callback for a periodic timer event, updating our pose estimate based on the encoders' ticks
void updatePose(void *data) {
    // calculate the distance each wheel has turned
    float leftDist = GetEncoderTicks(leftEncIndex) / ticksPerInch,
          rightDist = GetEncoderTicks(rightEncIndex) / ticksPerInch,
          x = pose.x,
          y = pose.y,
          heading = pose.heading,
          new_x,
          new_y,
          new_heading;

    // The following math represents the kinematics of differential steering 
    if (floatBasicallyEqual(leftDist, rightDist)) {
        new_x = x + leftDist * cos(heading);
        new_y = y + rightDist * sin(heading);
        new_heading = heading;
    } else {
        float R = inchesAxisWidth * (leftDist + rightDist) / (2 * (rightDist - leftDist)),
              wd = (rightDist - leftDist) / inchesAxisWidth;

        new_x = x + R * sin(wd + heading) - R * sin(heading);
        new_y = y - R * cos(wd + heading) + R * cos(heading);
        new_heading = boundAngle(heading + wd); 
    }
    
    // update the visible pose
    pose.x = new_x;
    pose.y = new_y;
    pose.heading = new_heading;
}

void setCurrentPose(RobotPose *_pose) {
    if (!_pose) {
        return;
    }

    pose.x = _pose->x;
    pose.y = _pose->y;
    pose.heading = _pose->heading;
}

void getCurrentPose(RobotPose *_pose) {
    if (!_pose) {
        return;
    }

    _pose->x = pose.x;
    _pose->y = pose.y;
    _pose->heading = pose.heading; 
}

void initDeadReckoning(
    RobotPose *initialPose,
    float _inchesAxisWidth,
    float _ticksPerInch,
    float timeStep,
    unsigned char _leftEncIndex,
    unsigned char _rightEncIndex
    )
{
    // ensure that this function is only executed once
    if (isInitialized) {
        return;
    }

    if (!initialPose) {
        // if no initial pose is provided, zero-out our pose
        pose.x = 0;
        pose.y = 0;
        pose.heading = 0;
    } else {
        // otherwise, copy the initialPose's members over to our pose
        pose.x = initialPose->x;
        pose.y = initialPose->y;
        pose.heading = initialPose->heading;
    }
    
    inchesAxisWidth = _inchesAxisWidth;
    ticksPerInch = _ticksPerInch;

    // we assume that these encoders have already been initialized 
    // (TODO: ensure that they have been initialized)
    leftEncIndex = _leftEncIndex;
    rightEncIndex = _rightEncIndex;

    // start a periodic timer event on behalf of the caller to update our pose using the encoders
    // (we don't set 'data' since we have what we need in this file's scope)    
    CallEvery(updatePose, 0, timeStep);

    isInitialized = 1;
}

// TODO: add tests
