package pose_estimator.simulation;

import java.util.List;

import config.CameraConfig;
import field.FieldMap;
import gtsam.Cal3DS2;
import gtsam.PinholeCamera;
import gtsam.Point2;
import gtsam.Point3;
import gtsam.Pose2;
import gtsam.Pose3;
import kinodynamics.Odometry;
import kinodynamics.Odometry.PointR2;
import kinodynamics.Odometry.Twist2d;

/**
 * see pose_estimator/simulation/circle_simulator.py in all24
 */
public class CircleSimulator {
    static double PATH_CENTER_X_M = 4;
    static double PATH_CENTER_Y_M = 4;
    static double PATH_RADIUS_M = 1;
    static double PATH_PERIOD_S = 2.0 * Math.PI;
    static double PAN_PERIOD_S = PATH_PERIOD_S / 3;
    // maximum pan angle, radians
    static double PAN_SCALE_RAD = 1.0;

    final FieldMap fieldMap;
    final Odometry.SwerveDriveKinematics100 kinematics;
    public Odometry.SwerveModulePositions positions;
    Pose2 wpi_pose = new Pose2(PATH_CENTER_X_M + PATH_RADIUS_M, 0, 0);

    double time_s = 0;
    public double gt_x;
    public double gt_y;
    public double gt_theta;

    // the order is the same as the detector getCorners order
    // lower left
    // lower right
    // upper right
    // upper left
    public List<Point2> gt_pixels;

    Point3 l0;
    Point3 l1;
    Point3 l2;
    Point3 l3;
    public List<Point3> landmarks;
    public Pose3 camera_offset;
    public Cal3DS2 calib;

    public CircleSimulator(FieldMap fieldMap) throws Throwable {
        this.fieldMap = fieldMap;

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

        // cheating the initial pose
        // TODO: more clever init
        Pose2 wpi_pose = new Pose2(PATH_CENTER_X_M + PATH_RADIUS_M, 0, 0);

        // constant landmark points
        // tag zero is at (3, 0, 1)
        List<Point3> tag = fieldMap.get(0);

        l0 = tag.get(0);
        l1 = tag.get(1);
        l2 = tag.get(2);
        l3 = tag.get(3);
        landmarks = List.of(l0, l1, l2, l3);

        CameraConfig cam = new CameraConfig();
        camera_offset = cam.camera_offset;
        calib = cam.calib;

        // initialize
        step(0);
    }

    /**
     * set all the state according to the supplied time
     */
    public void step(double dt_s) throws Throwable {
        time_s += dt_s;
        gt_x = PATH_CENTER_X_M + PATH_RADIUS_M * Math.cos(
                2 * Math.PI * time_s / PATH_PERIOD_S);
        gt_y = PATH_CENTER_Y_M + PATH_RADIUS_M * Math.sin(
                2 * Math.PI * time_s / PATH_PERIOD_S);
        gt_theta = PAN_SCALE_RAD * Math.sin(
                2 * Math.PI * time_s / PAN_PERIOD_S);

        // Find the wheel positions.
        Pose2 new_wpi_pose = new Pose2(gt_x, gt_y, gt_theta);
        Twist2d twist = Twist2d.fromVector(wpi_pose.log(new_wpi_pose));
        wpi_pose = new_wpi_pose;
        positions = kinematics.to_swerve_module_positions(positions, twist);

        Pose2 robot_pose = new Pose2(gt_x, gt_y, gt_theta);

        // lower left
        Point2 p0 = _px(
                l0,
                robot_pose,
                camera_offset,
                calib);
        // lower right
        Point2 p1 = _px(
                l1,
                robot_pose,
                camera_offset,
                calib);
        // upper right
        Point2 p2 = _px(
                l2,
                robot_pose,
                camera_offset,
                calib);
        // upper left
        Point2 p3 = _px(
                l3,
                robot_pose,
                camera_offset,
                calib);
        gt_pixels = List.of(p0, p1, p2, p3);

        // omit out-of-frame tags
        for (Point2 p : gt_pixels) {
            double x = p.x();
            double y = p.y();
            if (x < 0 || y < 0 || x > 800 || y > 600)
                gt_pixels = List.of();
        }
    }

    /**
     * Project the landmark point into the camera frame and return (x, y) in pixels.
     * Robot_pose and camera_offset are x-forward, z-up.
     */
    Point2 _px(
            Point3 landmark,
            Pose2 robot_pose,
            Pose3 camera_offset,
            Cal3DS2 calib) throws Throwable {
        Pose3 camera_pose = new Pose3(robot_pose).compose(camera_offset);
        PinholeCamera<Cal3DS2> camera = PinholeCamera.PinholeCameraCal3DS2(
                camera_pose, calib);
        return camera.project(landmark);
    }
}
