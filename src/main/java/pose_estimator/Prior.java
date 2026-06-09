package pose_estimator;

import gtsam.Key;
import gtsam.Pose2;
import gtsam.PriorFactor;
import gtsam.shared_ptr;
import gtsam.noiseModel.Base;

/**
 * A prior is a unary factor.
 * 
 * If there is no factor attached to a variable, or if only one component of a
 * variable is constrained (e.g. yaw but not x or y) the solver will fail.
 * In that case, use a very noisy prior.
 */
public class Prior {
    private final Solver estimate;

    public Prior(Solver e) {
        estimate = e;
    }

    /**
     * Add a prior. Can have wide noise model (when we really don't know)
     * or narrow (for resetting) or mixed (to reset rotation alone)
     * 
     * @param value is copied, ok to delete
     */
    public void add(
            long time_us,
            Pose2 value,
            shared_ptr<? extends Base> noise) throws Throwable {
        estimate.add(PriorFactor.PriorFactorPose2(
                Key.X(time_us), value, noise));
    }

}
