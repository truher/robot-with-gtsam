package pose_estimator;

import java.util.List;

import gtsam.BetweenFactorPose2;
import gtsam.Key;
import gtsam.Pose2;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Base;
import gtsam.noiseModel.Diagonal;
import kinodynamics.DriveUtil;
import kinodynamics.Kinematics;
import kinodynamics.Kinematics.PointR2;

public class Odometry {

    private final Estimate estimate;

    private final Kinematics.SwerveDriveKinematics100 kinematics;
    private Kinematics.SwerveModulePositions positions;

    Long t0_us = null;
    Pose2 default_prior;
    shared_ptr<? extends Base> default_prior_noise;

    public Odometry(Estimate e) throws Throwable {
        estimate = e;
        kinematics = new Kinematics.SwerveDriveKinematics100(
                List.of(
                        new PointR2(0.5, 0.5),
                        new PointR2(0.5, -0.5),
                        new PointR2(-0.5, 0.5),
                        new PointR2(-0.5, -0.5)));

        positions = new Kinematics.SwerveModulePositions(
                new Kinematics.SwerveModulePosition100(
                        0, new Kinematics.RotR2(1, 0)),
                new Kinematics.SwerveModulePosition100(
                        0, new Kinematics.RotR2(1, 0)),
                new Kinematics.SwerveModulePosition100(
                        0, new Kinematics.RotR2(1, 0)),
                new Kinematics.SwerveModulePosition100(
                        0, new Kinematics.RotR2(1, 0)));

        // for when we make a state but don't have any odometry for it
        default_prior = new Pose2(0, 0, 0);
        default_prior_noise = Diagonal.Sigmas(new Vector3(10, 10, 10));
    }

    /**
     * Add an odometry measurement. Remember to call add_state so that
     * the odometry factor has something to refer to.
     * 
     * t0_us, t1_us: network tables timestamp in integer microseconds.
     * TODO: something more clever with timestamps
     * 
     * TODO: noise should be speed dependent: when not moving, noise is very low,
     * and when moving fast, noise is much higher.
     */
    public void add(
            long t1_us,
            Kinematics.SwerveModulePositions newPositions,
            shared_ptr<? extends Base> noise) throws Throwable {

        // each odometry update maps exactly to a "between" factor
        // remember a "twist" is a robot-relative concept

        // print("odo time ", t1_us)
        if (t0_us == null) {
            // no previous state to refer to.
            // if this happens then the current state will likely
            // have no factors, so add a prior
            estimate.prior(t1_us, default_prior, default_prior_noise);
            // print("odo_t null")
            this.positions = newPositions;
            t0_us = t1_us;
            return;
        }

        Kinematics.SwerveModuleDeltas deltas = DriveUtil.module_position_delta(
                this.positions, newPositions);
        // this is the tangent-space (twist) measurement
        Kinematics.Twist2d measurement = kinematics.to_twist_2d(deltas);
        // print("add odometry factor ", t0_us, t1_us, self.measurement)
        Pose2 gp = new Pose2().expmap(new Vector3(
                measurement.x(),
                measurement.y(),
                measurement.theta()));

        estimate.add(BetweenFactorPose2.newBetweenFactorPose2(
                Key.X(t0_us), Key.X(t1_us), gp, noise));

        this.positions = newPositions;
        t0_us = t1_us;
    }

}
