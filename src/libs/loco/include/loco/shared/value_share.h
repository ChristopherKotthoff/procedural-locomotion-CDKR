#ifndef VALUE_SHARE_H
#define VALUE_SHARE_H

double* targetForwardSpeed_shared;
double maxSpeed = 2.0; // m/s
double initStrideDuration = 1.0 / 1.2; // eyeballed from https://simplifaster.com/articles/build-perfect-stride-cadence-runscribe/
double initSpeed = 2.5; // Below this, we just assume initStepDuration
double strideDurationSlope = -0.04386; // eyeballed from https://simplifaster.com/articles/build-perfect-stride-cadence-runscribe/

//crl::gui::SizeableGroundModel* ground_shared;



#endif // VALUE_SHARE_H