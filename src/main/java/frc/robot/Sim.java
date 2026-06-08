package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import field.FieldMap;
import gtsam.Key;
import gtsam.Pose2;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Base;
import gtsam.noiseModel.Diagonal;
import pose_estimator.Estimate;
import pose_estimator.simulation.CircleSimulator;
import util.Stats;

/**
 * Outer simulation loop.  Call "run" periodically.
 */
public class Sim {
    private final CircleSimulator sim;
    private final Estimate est;
    private final shared_ptr<? extends Base> odometry_noise;
    private final Stats etStats;
    private final Stats sizeStats;
    private final Field2d m_field;
    private final boolean initialized;

    private Pose2 state;
    private int loopCount;

    public Sim() {

        CircleSimulator sim = null;
        Estimate est = null;
        shared_ptr<Diagonal> odometry_noise = null;
        Pose2 state = null;
        boolean initialized = false;

        Field2d field = null;

        try {
            FieldMap fieldMap = new FieldMap();
            // TODO: correct tag location
            field = new Field2d();
            field.getObject("tag0").setPose(new Pose2d(8, 4, new Rotation2d(0)));
            sim = new CircleSimulator(fieldMap);
            int lagMicroseconds = 100000;
            est = new Estimate(lagMicroseconds);

            Pose2 prior_mean = new Pose2(0, 0, 0);
            est.add_state(0, prior_mean);
            est.prior(0, prior_mean, Diagonal.Sigmas(
                    new Vector3(100, 100, 100)));

            /** TODO: make odometry noise speed-dependent (not this constant). */
            odometry_noise = Diagonal.Sigmas(
                    new Vector3(0.02, 0.02, 0.05));
            // this should just record the positions and timestamp
            est.odometry(0, sim.positions(), odometry_noise);

            state = new Pose2();
            initialized = true;

        } catch (Throwable e) {
            e.printStackTrace();
        }

        this.sim = sim;
        this.est = est;
        this.odometry_noise = odometry_noise;
        this.state = state;
        m_field = field;
        etStats = new Stats();
        sizeStats = new Stats();
        loopCount = 1;

        SmartDashboard.putData("Field", m_field);
        this.initialized = initialized;
        // TODO: is this needed?
        run();
    }

    public void run() {
        if (!initialized)
            return;
        try {
            SmartDashboard.putNumber("i", loopCount);

            long t0_ns = System.nanoTime();

            long t1_us = 20000 * loopCount;

            //////////////////////////////////////////
            //
            // SIMULATE
            //
            // Update ground truth.
            sim.step(0.02);
            double gt_x = sim.gt_x;
            double gt_y = sim.gt_y;
            double gt_theta = sim.gt_theta;
            Pose2d gtPose2d = new Pose2d(gt_x, gt_y, new Rotation2d(gt_theta));
            m_field.getObject("gt").setPose(gtPose2d);

            //////////////////////////////////////////
            //
            // ESTIMATE
            //

            // Add the initial estimate of pose.
            est.add_state(t1_us, state);
            //
            est.odometry(t1_us, sim.positions(), odometry_noise);
            est.gyro(t1_us, sim.gt_theta);
            int pixelsInView = sim.gt_pixels.size();
            SmartDashboard.putNumber("pixels in view", pixelsInView);
            if (pixelsInView > 0) {
                est.apriltag_for_smoothing_batch(
                        sim.landmarks, sim.gt_pixels, t1_us, sim.camera_offset, sim.calib());
            }
            est.update();
            long t1_ns = System.nanoTime();
            long et_ns = t1_ns - t0_ns;
            etStats.update(et_ns);
            sizeStats.update(est.result_size());

            Key poseKey = Key.X(t1_us);
            Pose2 estPose2 = est.mean_pose2(poseKey);

            // Use the previous estimate as the new estimate.
            state = estPose2;

            double est_x = estPose2.x();
            double est_y = estPose2.y();
            double est_theta = estPose2.theta();

            Pose2d estPose2d = new Pose2d(est_x, est_y, new Rotation2d(est_theta));
            m_field.setRobotPose(estPose2d);

            double err_x = est_x - gt_x;
            double err_y = est_y - gt_y;
            double err_theta = est_theta - gt_theta;

            SmartDashboard.putNumber("err_x (m)", err_x);
            SmartDashboard.putNumber("err_y (m)", err_y);
            SmartDashboard.putNumber("err_theta (rad)", err_theta);
            SmartDashboard.putNumber("et (ms)", etStats.mean() / 1000000);
            SmartDashboard.putNumber("size", sizeStats.mean());

            // plot some samples around the mean.

            int N = 10;
            List<Pose2d> samples = new ArrayList<>();
            for (int i = 0; i < N; ++i) {
                Pose2 sample = est.sample_Pose2(poseKey);
                Pose2d wSample = toPose2d(sample);
                samples.add(wSample);
            }
            FieldObject2d o = m_field.getObject("samples");
            o.setPoses(samples);

            ++loopCount;
        } catch (Throwable e) {
            e.printStackTrace();
        }
    }

    Pose2d toPose2d(Pose2 p) throws Throwable {
        return new Pose2d(p.x(), p.y(), new Rotation2d(p.theta()));
    }
}
