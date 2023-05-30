#ifndef VALUE_SHARE_H
#define VALUE_SHARE_H

double* targetForwardSpeed_shared;
double maxSpeed = 4; // m/s
double initStrideDuration = 0.8; // eyeballed from https://simplifaster.com/articles/build-perfect-stride-cadence-runscribe/
double initSpeed = 2.5; // Below this, we just assume initStepDuration
double strideDurationSlope = -0.0; // eyeballed from https://simplifaster.com/articles/build-perfect-stride-cadence-runscribe/

//crl::gui::SizeableGroundModel* ground_shared;



#endif // VALUE_SHARE_H