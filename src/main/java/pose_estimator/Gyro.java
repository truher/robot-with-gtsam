package pose_estimator;

import gtsam.Key;
import gtsam.Pose2;
import gtsam.PoseRotationPrior;
import gtsam.Vector1;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

public class Gyro {
    private final Estimate estimate;
    private final shared_ptr<Diagonal> PRIOR_NOISE;
    private final Pose2 PRIOR_MEAN;
    private final shared_ptr<Diagonal> GYRO_NOISE;

    public Gyro(Estimate e) throws Throwable {
        estimate = e;
        PRIOR_NOISE = Diagonal.Sigmas(
                new Vector3(160, 80, 60));
        PRIOR_MEAN = new Pose2(8, 4, 0);
        GYRO_NOISE = Diagonal.Sigmas(
                new Vector1(0.01));
    }

    /**
     * TODO: use the new planar gyro factor.
     */
    public void add(long t0_us, double yaw) throws Throwable {
        // if this is the only factor attached to this variable
        // then it will be underconstrained (i.e. no constraint on x or y), which could
        // happen.
        estimate.add(
                PoseRotationPrior.PoseRotationPriorPose2(
                        Key.X(t0_us), new Pose2(0, 0, yaw), GYRO_NOISE));
        // if you have only the gyro (which only constrains yaw)
        // you will fail, so add an extremely loose prior.
        // TODO: get the caller to do this
        estimate.prior(t0_us, PRIOR_MEAN, PRIOR_NOISE);
    }

}
