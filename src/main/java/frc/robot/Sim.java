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
import gtsam.Vector;
import gtsam.Vector1;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;
import kinodynamics.Kinematics.SwerveModulePositions;
import pose_estimator.BetweenGyro;
import pose_estimator.Gyro;
import pose_estimator.Odometry;
import pose_estimator.Prior;
import pose_estimator.Solver;
import pose_estimator.Vision;
import simulation.SimulatedCamera;
import simulation.SimulatedGyro;
import simulation.SimulatedOdometry;
import simulation.SimulatedRobot;
import util.Geometry;

/**
 * Outer simulation loop. Call "run" periodically.
 */
public class Sim {
    private static final boolean USE_GYRO = true;
    private static final boolean NEW_GYRO = true;
    private static final boolean USE_ODO = true;
    private static final boolean USE_VISION = true;
    private final Solver m_solver;
    private final Field2d m_field;
    private final List<Point3> m_landmarks;

    // simulated measurements
    private final SimulatedOdometry m_simulatedOdometry;
    private final SimulatedRobot m_simulatedRobot;
    private final SimulatedCamera m_simulatedCamera;
    private final SimulatedGyro m_simulatedGyro;

    // factors
    private final Vision m_vision;
    private final Gyro m_gyro;
    private final BetweenGyro m_betweenGyro;
    private final Odometry m_odometry;
    private final Prior m_prior;
    private final boolean m_initialized;

    private Pose2d m_groundTruthPose;
    /** Estimate from the solver. */
    private Pose2 m_estimatedPose;
    private Vector m_poseSigma;
    private int m_loopCount;

    // the verbosity here is to trap the exception.
    public Sim() {

        Solver solver = null;
        Pose2d groundTruthPose = null;
        Pose2 estimatedPose = null;
        Vector poseSigma = null;
        boolean initialized = false;
        List<Point3> landmarks = null;
        CameraConfig conf = null;

        // SIMULATED MEASUREMENTS
        SimulatedRobot simulatedRobot = null;
        SimulatedOdometry simulatedOdometry = null;
        SimulatedCamera simulatedCamera = null;
        SimulatedGyro simulatedGyro = null;

        // FACTORS
        Vision vision = null;
        Gyro gyro = null;
        BetweenGyro betweenGyro = null;
        Odometry odometry = null;
        Prior prior = null;

        Field2d field = null;

        try {

            int lagMicroseconds = 100000;
            solver = new Solver(lagMicroseconds);

            groundTruthPose = new Pose2d();
            estimatedPose = new Pose2();
            poseSigma = new Vector(3);

            //
            // LANDMARKS
            //
            List<Point3> tag = new FieldMap().get(0);
            landmarks = List.of(tag.get(0), tag.get(1), tag.get(2), tag.get(3));
            FieldMap fieldMap = new FieldMap();
            field = new Field2d();
            field.getObject("tag0").setPose(new Pose2d(tag.get(0).x(), tag.get(0).y(), new Rotation2d(0)));
            field.getObject("tag1").setPose(new Pose2d(tag.get(1).x(), tag.get(1).y(), new Rotation2d(0)));
            field.getObject("tag2").setPose(new Pose2d(tag.get(2).x(), tag.get(2).y(), new Rotation2d(0)));
            field.getObject("tag3").setPose(new Pose2d(tag.get(3).x(), tag.get(3).y(), new Rotation2d(0)));

            conf = new CameraConfig();

            //
            // SIMULATED MEASUREMENTS
            //
            simulatedRobot = new SimulatedRobot();
            Pose2d initial = simulatedRobot.pose(0);
            simulatedCamera = new SimulatedCamera(landmarks, conf);
            simulatedGyro = new SimulatedGyro(NEW_GYRO);

            //
            // FACTORS
            //
            vision = new Vision(solver, conf);
            gyro = new Gyro(solver);
            betweenGyro = new BetweenGyro(solver);
            odometry = new Odometry(solver);
            prior = new Prior(solver);

            // Initial pose.
            // Pose2 p0 = new Pose2(0, 0, 0);
            
            // Cheat: use the real ground truth pose as the initial pose
            Pose2 p0 = Geometry.toPose2(initial);
            Key x0 = Key.X(0);
            solver.addVariable(x0, 0, p0);
            // Very uncertain prior, let the solver figure it out.
            // shared_ptr<Diagonal> priorNoise = Diagonal.Sigmas(new Vector3(100, 100,
            // 100));
            // Very tight prior to help the solver.
            shared_ptr<Diagonal> priorNoise = Diagonal.Sigmas(new Vector3(0.1, 0.1, 0.1));
            prior.add(x0, p0, priorNoise);

            // Initial gyro bias.
            if (USE_GYRO) {
                if (NEW_GYRO) {
                    Key b0 = Key.B(0);
                    solver.addVariable(b0, 0, 0);
                    // This is a very wide bias prior.
                    prior.add(b0, 0, Diagonal.Sigmas(new Vector1(1)));
                    betweenGyro.add(0, simulatedGyro.yaw(0, initial));
                }
            }

            // Record the initial timestamp and positions.
            simulatedOdometry = new SimulatedOdometry(fieldMap, initial);
            if (USE_ODO) {
                odometry.add(0, simulatedOdometry.positions(initial));
            }

            initialized = true;
        } catch (Throwable e) {
            e.printStackTrace();
        }

        m_solver = solver;
        m_groundTruthPose = groundTruthPose;
        m_estimatedPose = estimatedPose;
        m_poseSigma = poseSigma;
        m_field = field;
        m_loopCount = 1;
        m_landmarks = landmarks;
        // simulated measurements
        m_simulatedCamera = simulatedCamera;
        m_simulatedGyro = simulatedGyro;
        m_simulatedRobot = simulatedRobot;
        m_simulatedOdometry = simulatedOdometry;
        // factors
        m_vision = vision;
        m_gyro = gyro;
        m_betweenGyro = betweenGyro;
        m_odometry = odometry;
        m_prior = prior;
        SmartDashboard.putData("Field", m_field);
        m_initialized = initialized;
    }

    public void run() {
        if (!m_initialized)
            return;
        try {
            SmartDashboard.putNumber("i", m_loopCount);

            // Nanosecond timer to see how long the solver takes.
            long t0_ns = System.nanoTime();

            // Current simulation time in microseconds.
            long t1_us = 20000 * m_loopCount;

            // Compute ground truth and plot it.
            m_groundTruthPose = m_simulatedRobot.pose(t1_us);
            m_field.getObject("gt").setPose(m_groundTruthPose);

            // System.out.println("==> Initial value is the previous estimate.");
            Key x1 = Key.X(t1_us);
            // m_solver.addVariable(x1, t1_us, m_estimatedPose);

            // Cheat by setting the initial value to the true value.
            // This makes the incremental solver work.
            m_solver.addVariable(x1, t1_us, Geometry.toPose2(m_groundTruthPose));

            // System.out.println("==> Add odometry factors.");
            if (USE_ODO) {
                applyOdometry(t1_us, m_groundTruthPose);
            }

            // System.out.println("==> Add gyro factors.");
            if (USE_GYRO) {
                if (NEW_GYRO) {
                    applyBetweenGyro(t1_us, m_groundTruthPose);
                } else {
                    applyGyro(t1_us, m_groundTruthPose);
                }
            }

            // System.out.println("==> Add camera factors.");
            if (USE_VISION) {
                applyCamera(t1_us, m_groundTruthPose);
            }

            // System.out.println("==> Run the solver.");
            m_solver.update();

            // System.out.println("==> Log a little about the iteration.");
            logET(t0_ns);

            // System.out.println("==> Retrieve the estimated pose.");
            m_estimatedPose = m_solver.mean_pose2(x1);

            // System.out.println("==> Show the estimate, and errors.");
            plotEstimatedPose(m_groundTruthPose);
            // System.out.println("==> Show samples on the field.");
            plotSamples(t1_us);

            // System.out.println("==> Show the estimated bias.");

            if (USE_GYRO) {
                if (NEW_GYRO) {
                    double b = m_solver.mean_double(Key.B(t1_us));
                    SmartDashboard.putNumber("bias", b);
                }
            }

            m_poseSigma = m_solver.sigma_pose2(x1);
            SmartDashboard.putNumber("pose sigma x (m)", m_poseSigma.at(0));
            SmartDashboard.putNumber("pose sigma y (m)", m_poseSigma.at(1));
            SmartDashboard.putNumber("pose sigma (rad)", m_poseSigma.at(2));

            if (USE_GYRO) {
                if (NEW_GYRO) {
                    Vector biasSigma = m_solver.sigma_pose2(Key.B(t1_us));
                    SmartDashboard.putNumber("bias sigma (rad)", biasSigma.at(0));
                }
            }
            ++m_loopCount;
        } catch (Throwable e) {
            e.printStackTrace();
        }
    }

    public Pose2d groundTruthPose() {
        return m_groundTruthPose;
    }

    public Pose2 estimatedPose() {
        return m_estimatedPose;
    }

    public Vector poseSigma() {
        return m_poseSigma;
    }

    /** Plot the estimated pose and the error from ground truth. */
    private void plotEstimatedPose(Pose2d groundTruthPose) throws Throwable {
        Pose2d estPose2d = new Pose2d(m_estimatedPose.x(), m_estimatedPose.y(),
                new Rotation2d(m_estimatedPose.theta()));
        m_field.setRobotPose(estPose2d);
        logErr(groundTruthPose, estPose2d);
    }

    private void logET(long t0_ns) throws Throwable {
        long t1_ns = System.nanoTime();
        long et_ns = t1_ns - t0_ns;
        SmartDashboard.putNumber("et (ms)", (double) et_ns * 1e-6);
        SmartDashboard.putNumber("size", m_solver.result_size());
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
            Pose2 sample = m_solver.sample_Pose2(Key.X(t1_us));
            Pose2d wSample = Geometry.toPose2d(sample);
            samples.add(wSample);
        }
        FieldObject2d o = m_field.getObject("samples");
        o.setPoses(samples);
    }

    private void applyGyro(long t1_us, Pose2d gtPose2d) throws Throwable {
        double measurement = m_simulatedGyro.yaw(t1_us, gtPose2d);
        m_gyro.add(t1_us, measurement);
    }

    private void applyBetweenGyro(long t1_us, Pose2d gtPose2d) throws Throwable {
        double measurement = m_simulatedGyro.yaw(t1_us, gtPose2d);
        Key b = Key.B(t1_us);
        m_solver.addVariable(b, t1_us, 0);
        m_betweenGyro.add(t1_us, measurement);
    }

    private void applyOdometry(long t1_us, Pose2d gtPose2d) throws Throwable {
        SwerveModulePositions positions = m_simulatedOdometry.positions(gtPose2d);
        m_odometry.add(t1_us, positions);
    }

    /** Retrieve simulated camera measurements and apply them to the graph. */
    private void applyCamera(long t1_us, Pose2d gtPose2d) throws Throwable {
        List<Point2> measurements = m_simulatedCamera.pixels(gtPose2d);
        if (m_landmarks.size() != measurements.size())
            return;
        // try one point only, for debugging.
        // m_vision.add(t1_us, m_landmarks.get(0), measurements.get(0));
        for (int i = 0; i < m_landmarks.size(); ++i) {
            Point3 landmark = m_landmarks.get(i);
            Point2 measurement = measurements.get(i);
            m_vision.add(t1_us, landmark, measurement);
        }
    }

}
