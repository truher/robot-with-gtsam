package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import field.FieldMap;
import gtsam.Key;
import gtsam.Point3;
import gtsam.Pose2;
import gtsam.SharedNoiseModel;
import gtsam.Vector3;
import pose_estimator.Estimate;
import pose_estimator.simulation.CircleSimulator;
import util.Stats;

public class Sim {

    private final CircleSimulator sim;
    private final Estimate est;
    private final SharedNoiseModel odometry_noise;
    private final Stats etStats;
    private final Stats sizeStats;
    private final Field2d m_field;

    private Pose2 state;
    private int i;

    public Sim(
            Field2d field,
            CircleSimulator sim,
            Estimate est,
            SharedNoiseModel odometry_noise,
            Pose2 state) {
        this.sim = sim;
        this.est = est;
        this.odometry_noise = odometry_noise;
        this.state = state;
        m_field = field;
        etStats = new Stats();
        sizeStats = new Stats();
        i = 1;

        SmartDashboard.putData("Field", m_field);
        // Do this in either robot periodic or subsystem periodic
    }

    public static Sim make() {
        try {

            FieldMap fieldMap = new FieldMap();
            // TODO: correct tag location
            Field2d field = new Field2d();
            field.getObject("tag0").setPose(new Pose2d(8, 4, new Rotation2d(0)));
            CircleSimulator sim = new CircleSimulator(fieldMap);
            Estimate est = new Estimate(100000);
            est.init();

            Pose2 prior_mean = new Pose2(0, 0, 0);
            est.add_state(0, prior_mean);
            est.prior(0, prior_mean, SharedNoiseModel.Sigmas(
                    new Vector3(100, 100, 100)));

            SharedNoiseModel odometry_noise = SharedNoiseModel.Sigmas(
                    new Vector3(0.01, 0.01, 0.01));
            // this should just record the positions and timestamp
            est.odometry(0, sim.positions, odometry_noise);

            Pose2 state = new Pose2();

            return new Sim(field, sim, est, odometry_noise, state);
        } catch (Throwable e) {
            e.printStackTrace();
            return null;
        }
    }

    public void run() throws Throwable {
        long t0_ns = System.nanoTime();

        long t1_us = 20000 * i;
        // Update ground truth.
        sim.step(0.02);
        // Add the initial estimate of pose.
        est.add_state(t1_us, state);
        // 
        est.odometry(t1_us, sim.positions, odometry_noise);
        est.gyro(t1_us, sim.gt_theta);

        if (sim.gt_pixels.size() > 0) {
            est.apriltag_for_smoothing_batch(
                    sim.landmarks, sim.gt_pixels, t1_us, sim.camera_offset, sim.calib);
        }
        est.update();
        long t1_ns = System.nanoTime();
        long et_ns = t1_ns - t0_ns;
        etStats.update(et_ns);
        sizeStats.update(est.result_size());

        double gt_x = sim.gt_x;
        double gt_y = sim.gt_y;
        double gt_theta = sim.gt_theta;

        Pose2 estPose2 = est.mean_pose2(Key.X(t1_us));
        // Use the previous estimate as the new estimate.
        state = estPose2;
        double est_x = estPose2.x();
        double est_y = estPose2.y();
        double est_theta = estPose2.theta();

        Pose2d estPose2d = new Pose2d(est_x, est_y, new Rotation2d(est_theta));
        Pose2d gtPose2d = new Pose2d(gt_x, gt_y, new Rotation2d(gt_theta));

        m_field.setRobotPose(estPose2d);
        m_field.getObject("gt").setPose(gtPose2d);

        double err_x = est_x - gt_x;
        double err_y = est_y - gt_y;
        double err_theta = est_theta - gt_theta;

        SmartDashboard.putNumber("err_x (mm)", err_x / 1000);
        SmartDashboard.putNumber("err_y (mm)", err_y / 1000);
        SmartDashboard.putNumber("err_theta (mrad)", err_theta / 1000);
        SmartDashboard.putNumber("et (ms)", etStats.mean() / 1000000);
        SmartDashboard.putNumber("size", sizeStats.mean());

        ++i;
    }
}
