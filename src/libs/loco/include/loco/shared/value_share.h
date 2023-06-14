#ifndef VALUE_SHARE_H
#define VALUE_SHARE_H

double* targetForwardSpeed_shared;

// m/s. No reliable measurements exist below this speed, so we use this as our initial speed.
double initSpeed = 0.6;

// m/s. The speed at which humans transition from walking to running according to Hansen et al. (2017).
double walkToRunTransitionSpeed = 2.1;

// m/s. We could go higher.
double maxSpeed = 10;

// According to Nilsson et al (1985).
double strideDurationSlopeWalk = -0.419;

// According to Nilsson et al (1985).
double strideDurationInterceptWalk = 1.927;

// According to Nilsson et al (1985).
double strideDurationSlopeRun = -0.041;

// According to Nilsson et al (1985).
double strideDurationInterceptRun = 0.901;

/*
* Returns the duration of a stride in seconds.
*/
double strideDurationInSeconds(const double speed) {
    if (speed < walkToRunTransitionSpeed) {
        return strideDurationSlopeWalk * speed + strideDurationInterceptWalk;
    } else {
        return strideDurationSlopeRun * speed + strideDurationInterceptRun;
    }
}

/*
* Returns the duration of the swing phase relative to the whole stride duration.
*/
double swingPhaseDurationRelative(const double speed) {
    if (speed < initSpeed) {
        return (3.4 * initSpeed + 37.1) / 100.0; // Hansen et al. was in percent, so we divide by 100.
    } else if (speed > walkToRunTransitionSpeed) {
        return 0.6; // Beyond walking speed, we fix to 60%.
    } else {
        return (3.4 * speed + 37.1) / 100.0; // Hansen et al. was in percent, so we divide by 100.
    }
}

//crl::gui::SizeableGroundModel* ground_shared;



#endif // VALUE_SHARE_H