package pose_estimator;

import java.util.List;

import gtsam.BatchFixedLagSmoother;
import gtsam.BetweenFactorPose2;
import gtsam.Cal3DS2;
import gtsam.FixedLagSmoother;
import gtsam.Key;
import gtsam.Marginals;
import gtsam.Matrix;
import gtsam.NonlinearFactorGraph;
import gtsam.PlanarProjectionFactor1;
import gtsam.Point2;
import gtsam.Point3;
import gtsam.Pose2;
import gtsam.Pose3;
import gtsam.PoseRotationPrior;
import gtsam.PriorFactor;
import gtsam.SharedNoiseModel;
import gtsam.Values;
import gtsam.Vector;
import gtsam.Vector1;
import gtsam.Vector2;
import gtsam.Vector3;
import kinodynamics.DriveUtil;
import kinodynamics.Odometry;
import kinodynamics.Odometry.PointR2;

/** Port of estimate.py from 2024. */
public class Estimate {
    SharedNoiseModel PRIOR_NOISE = SharedNoiseModel.Sigmas(
            new Vector3(160, 80, 60));
    Pose2 PRIOR_MEAN = new Pose2(8, 4, 0);
    SharedNoiseModel GYRO_NOISE = SharedNoiseModel.Sigmas(
            new Vector1(0.001));

    private final BatchFixedLagSmoother isam;
    Values result;
    private final NonlinearFactorGraph new_factors;
    private final Values new_values;
    /** key is Key, "X(timestamp in us)", value is timestamp in us */
    private final FixedLagSmoother.KeyTimestampMap new_timestamps;
    final Odometry.SwerveDriveKinematics100 kinematics;
    public Odometry.SwerveModulePositions positions;

    Long odo_t = null;
    long odo_dt = 0;

    Pose2 default_prior;
    SharedNoiseModel default_prior_noise;
    Odometry.Twist2d measurement = new Odometry.Twist2d();

    /** @param lag in microseconds, not seconds as in python */
    public Estimate(double lag) throws Throwable {
        // Initialize the model
        // initial module positions are at their origins.
        // TODO: some other initial positions?

        isam = new BatchFixedLagSmoother(lag);
        result = new Values();
        // between updates we accumulate inputs here

        new_factors = new NonlinearFactorGraph();
        new_values = new Values();
        new_timestamps = new FixedLagSmoother.KeyTimestampMap();

        kinematics = new Odometry.SwerveDriveKinematics100(
                List.of(
                        new PointR2(0.5, 0.5),
                        new PointR2(0.5, -0.5),
                        new PointR2(-0.5, 0.5),
                        new PointR2(-0.5, -0.5)));
        positions = new Odometry.SwerveModulePositions(
                new Odometry.SwerveModulePosition100(
                        0, new Odometry.RotR2(1, 0)),
                new Odometry.SwerveModulePosition100(
                        0, new Odometry.RotR2(1, 0)),
                new Odometry.SwerveModulePosition100(
                        0, new Odometry.RotR2(1, 0)),
                new Odometry.SwerveModulePosition100(
                        0, new Odometry.RotR2(1, 0)));

        // for when we make a state but don't have any odometry for it
        default_prior = new Pose2(0, 0, 0);
        default_prior_noise = SharedNoiseModel.Sigmas(new Vector3(10, 10, 10));

    }

    public void init() {
        //
    }

    /**
     * Add a new robot state (pose) to the estimator, if it doesn't already exist.
     * 
     * @param time_us
     * @param initial_value cloned, ok to delete after this.
     */
    public void add_state(long time_us, Pose2 initial_value) throws Throwable {
        Key key = Key.X(time_us);
        // System.out.printf("add state %d\n", key.j);
        if (result.exists(key)) {
            // System.out.printf("Key %d is already in the model\n", key.j);
            return;
        }
        if (new_values.exists(key)) {
            // System.out.printf("Key %d is already in the values", key.j);
            return;
        }
        // if you're using the batch smoother, the initial value
        // almost doesn't matter:
        // TODO: use the previous pose as the initial value
        new_values.insert(key, initial_value);
        new_timestamps.put(key, time_us);
    }

    /**
     * Add a prior. Can have wide noise model (when we really don't know)
     * or narrow (for resetting) or mixed (to reset rotation alone)
     * 
     * @param value is copied, ok to delete
     */
    public void prior(
            long time_us,
            Pose2 value,
            SharedNoiseModel noise) throws Throwable {

        new_factors.add(
                PriorFactor.PriorFactorPose2(
                        Key.X(time_us),
                        value,
                        noise));
        // use this prior if there's a hole to fill
        default_prior = value;
        default_prior_noise = noise;
    }

    /**
     * Add an odometry measurement. Remember to call add_state so that
     * the odometry factor has something to refer to.
     * 
     * t0_us, t1_us: network tables timestamp in integer microseconds.
     * TODO: something more clever with timestamps
     */
    public void odometry(
            long t1_us,
            Odometry.SwerveModulePositions newPositions,
            SharedNoiseModel noise) throws Throwable {

        // each odometry update maps exactly to a "between" factor
        // remember a "twist" is a robot-relative concept

        // print("odo time ", t1_us)
        if (odo_t == null) {
            // no previous state to refer to.
            // if this happens then the current state will likely
            // have no factors, so add a prior
            prior(t1_us, default_prior, default_prior_noise);
            // print("odo_t null")
            this.positions = newPositions;
            odo_t = t1_us;
            return;
        }

        long t0_us = odo_t;

        Odometry.SwerveModuleDeltas deltas = DriveUtil.module_position_delta(
                this.positions, newPositions);
        // this is the tangent-space (twist) measurement
        Odometry.Twist2d measurement = kinematics.to_twist_2d(deltas);
        odo_dt = t1_us - t0_us;
        // print("add odometry factor ", t0_us, t1_us, self.measurement)
        Pose2 gp = Pose2.Expmap(new Vector3(
                measurement.x(),
                measurement.y(),
                measurement.theta()));

        new_factors.add(
                BetweenFactorPose2.newBetweenFactorPose2(
                        Key.X(t0_us), Key.X(t1_us), gp, noise));

        this.positions = newPositions;
        odo_t = t1_us;
    }

    /**
     * ALERT! The gyro in 2026 works like a "between" factor,
     * so this should be changed accordingly.
     */
    public void gyro(long t0_us, double yaw) throws Throwable {
        // if this is the only factor attached to this variable
        // then it will be underconstrained (i.e. no constraint on x or y), which could
        // happen.
        new_factors.add(
                PoseRotationPrior.PoseRotationPriorPose2(
                        Key.X(t0_us), new Pose2(0, 0, yaw), GYRO_NOISE));
        // if you have only the gyro (which only constrains yaw)
        // you will fail, so add an extremely loose prior.
        prior(t0_us, PRIOR_MEAN, PRIOR_NOISE);
    }

    /**
     * Add a factor for each landmark/pixel pair.
     */
    public void apriltag_for_smoothing_batch(
            List<Point3> landmarks,
            List<Point2> measured,
            long t0_us,
            Pose3 camera_offset,
            Cal3DS2 calib) throws Throwable {
        if (landmarks.size() != measured.size())
            throw new IllegalArgumentException();

        for (int i = 0; i < landmarks.size(); ++i) {
            Point3 landmark = landmarks.get(i);
            Point2 px = measured.get(i);
            SharedNoiseModel noise = SharedNoiseModel.Sigmas(
                    new Vector2(1, 1));
            new_factors.add(
                    PlanarProjectionFactor1.newPlanarProjectionFactor1(
                            Key.X(t0_us),
                            landmark,
                            px,
                            camera_offset,
                            calib,
                            noise));
        }
    }

    /**
     * Run the solver
     */
    public void update() throws Throwable {
        // System.out.println("============UPDATE============");
        // print(self._new_factors)
        // print(self._new_values)
        // new_values.print();
        // print(self._new_timestamps)
        isam.update(new_factors, new_values, new_timestamps);
        // System.out.println("retrieve estimates");
        result = isam.calculateEstimate();

        // print("TIMESTAMPS")
        // print(self.isam.timestamps())
        // k = max(isam.timestamps().keys());
        // ts = max(isam.timestamps().values());
        // print(self.result.atPose2(k))
        // print(ts)

        // reset the accumulators
        new_factors.resize(0);
        new_values.clear();
        new_timestamps.clear();
    }

    public long result_size() throws Throwable {
        // result.print();
        return result.size();
    }

    public Pose2 mean_pose2(Key key) throws Throwable {
        return result.atPose2(key);
    }

    public Vector sigma_pose2(Key key) throws Throwable {
        Marginals m = marginal_covariance();
        Matrix s = m.marginalCovariance(key);
        return s.diagonal_cwiseSqrt();
    }

    public Marginals marginal_covariance() throws Throwable {
        NonlinearFactorGraph factors = isam.getFactors();
        return new Marginals(factors, result);
    }

}
