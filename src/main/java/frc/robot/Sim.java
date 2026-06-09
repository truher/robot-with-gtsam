package frc.robot;

import java.util.ArrayList;
import java.util.List;

import config.CameraConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import field.FieldMap;
import gtsam.Key;
import gtsam.Point2;
import gtsam.Point3;
import gtsam.Pose2;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Base;
import gtsam.noiseModel.Diagonal;
import kinodynamics.Kinematics.SwerveModulePositions;
import pose_estimator.Estimate;
import pose_estimator.Gyro;
import pose_estimator.Odometry;
import pose_estimator.Vision;
import simulation.SimulatedCamera;
import simulation.SimulatedOdometry;
import simulation.SimulatedRobot;

/**
 * Outer simulation loop. Call "run" periodically.
 */
public class Sim {
    private final SimulatedOdometry sim;
    private final Estimate est;
    private final shared_ptr<? extends Base> odometry_noise;
    private final Field2d m_field;
    private final boolean initialized;
    private final List<Point3> landmarks;
    private final CameraConfig cameraconfig;
    private final SimulatedCamera camera;
    private final Vision vision;
    private final Gyro gyro;
    private final Odometry odometry;

    private Pose2 state;
    private int loopCount;

    public Sim() {

        SimulatedOdometry sim = null;
        Estimate est = null;
        shared_ptr<Diagonal> odometry_noise = null;
        Pose2 state = null;
        boolean initialized = false;
        List<Point3> lm = null;
        CameraConfig conf = null;
        SimulatedCamera cam = null;
        Vision viz = null;
        Gyro gy = null;
        Odometry od = null;

        Field2d field = null;

        try {
            FieldMap fieldMap = new FieldMap();
            // TODO: correct tag location
            field = new Field2d();
            field.getObject("tag0").setPose(new Pose2d(8, 4, new Rotation2d(0)));
            Pose2d initial = SimulatedRobot.pose(0);

            sim = new SimulatedOdometry(fieldMap, initial);
            int lagMicroseconds = 100000;
            est = new Estimate(lagMicroseconds);

            Pose2 prior_mean = new Pose2(0, 0, 0);
            est.addVariable(0, prior_mean);
            est.prior(0, prior_mean, Diagonal.Sigmas(
                    new Vector3(100, 100, 100)));

            /** TODO: make odometry noise speed-dependent (not this constant). */
            odometry_noise = Diagonal.Sigmas(
                    new Vector3(0.02, 0.02, 0.05));

            state = new Pose2();

            List<Point3> tag = new FieldMap().get(0);
            lm = List.of(tag.get(3), tag.get(1), tag.get(2), tag.get(3));

            conf = new CameraConfig();
            cam = new SimulatedCamera(lm, conf);

            viz = new Vision(est, conf);
            gy = new Gyro(est);
            od = new Odometry(est);

            // this should just record the positions and timestamp
            od.add(0, sim.positions(initial), odometry_noise);

            initialized = true;
        } catch (Throwable e) {
            e.printStackTrace();
        }

        this.sim = sim;
        this.est = est;
        this.odometry_noise = odometry_noise;
        this.state = state;
        m_field = field;
        loopCount = 1;
        landmarks = lm;
        cameraconfig = conf;
        camera = cam;
        vision = viz;
        gyro = gy;
        odometry = od;

        SmartDashboard.putData("Field", m_field);
        this.initialized = initialized;
    }

    public void run() {
        if (!initialized)
            return;
        try {
            SmartDashboard.putNumber("i", loopCount);

            // Nanosecond timer to see how long the solver takes.
            long t0_ns = System.nanoTime();

            // Current simulation time in microseconds.
            long t1_us = 20000 * loopCount;

            ////
            //
            // SIMULATE
            //
            // Update ground truth.

            Pose2d gtPose2d = SimulatedRobot.pose(t1_us);

            ////
            //
            // ESTIMATE
            //

            // Add the initial estimate of pose.

            est.addVariable(t1_us, state);

            applyOdometry(t1_us, gtPose2d);

            applyGyro(t1_us, gtPose2d);

            applyCamera(t1_us, gtPose2d);

            // Run the solver
            est.update();

            // Log a little about the iteration.
            long t1_ns = System.nanoTime();
            long et_ns = t1_ns - t0_ns;
            SmartDashboard.putNumber("et (ms)", (double) et_ns * 1e-6);
            SmartDashboard.putNumber("size", est.result_size());

            state = est.mean_pose2(Key.X(t1_us));

            Pose2d estPose2d = new Pose2d(state.x(), state.y(), new Rotation2d(state.theta()));
            m_field.setRobotPose(estPose2d);
            logErr(gtPose2d, estPose2d);
            plotSamples(t1_us);
            m_field.getObject("gt").setPose(gtPose2d);

            ++loopCount;
        } catch (Throwable e) {
            e.printStackTrace();
        }
    }

    /**
     * Log the estimation error.
     */
    private void logErr(Pose2d gtPose2d, Pose2d estPose2d) {
        Transform2d poseErr = estPose2d.minus(gtPose2d);
        SmartDashboard.putNumber("err_x (m)", poseErr.getX());
        SmartDashboard.putNumber("err_y (m)", poseErr.getY());
        SmartDashboard.putNumber("err_theta (rad)", poseErr.getRotation().getRadians());
    }

    /**
     * Plot some samples around the mean.
     */
    private void plotSamples(long t1_us) throws Throwable {
        int N = 10;
        List<Pose2d> samples = new ArrayList<>();
        for (int i = 0; i < N; ++i) {
            Pose2 sample = est.sample_Pose2(Key.X(t1_us));
            Pose2d wSample = toPose2d(sample);
            samples.add(wSample);
        }
        FieldObject2d o = m_field.getObject("samples");
        o.setPoses(samples);
    }

    private void applyGyro(long t1_us, Pose2d gtPose2d) throws Throwable {
        // est.gyro(t1_us, gtPose2d.getRotation().getRadians());
        gyro.add(t1_us, gtPose2d.getRotation().getRadians());
    }

    private void applyOdometry(long t1_us, Pose2d gtPose2d) throws Throwable {
        SwerveModulePositions positions = sim.positions(gtPose2d);
        // est.odometry(t1_us, positions, odometry_noise);
        odometry.add(t1_us, positions, odometry_noise);
    }

    /** Retrieve simulated camera measurements and apply them to the graph. */
    private void applyCamera(long t1_us, Pose2d gtPose2d) throws Throwable {
        List<Point2> measurements = camera.pixels(gtPose2d);
        if (landmarks.size() != measurements.size())
            return;
        for (int i = 0; i < landmarks.size(); ++i) {
            Point3 landmark = landmarks.get(i);
            Point2 measurement = measurements.get(i);
            vision.add(t1_us, landmark, measurement);
        }
    }

    Pose2d toPose2d(Pose2 p) throws Throwable {
        return new Pose2d(p.x(), p.y(), new Rotation2d(p.theta()));
    }
}
