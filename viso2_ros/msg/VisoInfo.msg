# Internal information on the
# viso2 algorithm parameters
# and results

Header header

# True if the previous iteration of viso2
# was not able to complete the matching process
# therefore the visual odometer is re-started.
bool got_lost

# info from motion estimator
bool motion_estimate_valid
int32 num_matches
int32 num_inliers

# runtime of last iteration in seconds
float64 runtime
