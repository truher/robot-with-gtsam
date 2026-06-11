package pose_estimator;

import config.CameraConfig;
import gtsam.Cal3DS2;
import gtsam.Key;
import gtsam.Matrix;
import gtsam.PlanarProjectionFactor1;
import gtsam.Point2;
import gtsam.Point3;
import gtsam.Pose2;
import gtsam.Pose3;
import gtsam.Vector2;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

/**
 * Use PlanarProjectionFactor to model camera measurements.
 */
public class Vision {
    private static final boolean DEBUG = false;
    private final Solver solver;
    private final Pose3 camera_offset;
    private final Cal3DS2 calib;
    private final shared_ptr<Diagonal> noise;

    public Vision(Solver solv, CameraConfig conf) throws Throwable {
        solver = solv;
        camera_offset = conf.camera_offset;
        calib = conf.calib;
        // pixel noise is small. mistakes in calibration will result
        // in larger errors, though.
        // This is a reasonable number: just a pixel or two.
        // noise = Diagonal.Sigmas(new Vector2(2, 2));
        // This is a very high noise to make the demo look better.
        noise = Diagonal.Sigmas(new Vector2(20, 20));
    }

    /**
     * Add a factor for the measurement of the landmark.
     */
    public void add(long t1_us, Point3 landmark, Point2 measurement)
            throws Throwable {
        if (DEBUG) {
            System.out.printf("landmark (%f %f %f)\n", landmark.x(), landmark.y(), landmark.z());
            System.out.printf("measurement (%f %f)\n", measurement.x(), measurement.y());
        }
        shared_ptr<PlanarProjectionFactor1> f = //
                PlanarProjectionFactor1.newPlanarProjectionFactor1(
                        Key.X(t1_us),
                        landmark,
                        measurement,
                        camera_offset,
                        calib,
                        noise);
        solver.add(f);
        // troubleshooting
        if (DEBUG) {
            Matrix H = new Matrix();
            Vector2 v = f.get().evaluateError(new Pose2(4, 5, 0), H);
            System.out.printf("err %f %f\n", v.at(0), v.at(1));
            H.print("H");
        }
    }

}
