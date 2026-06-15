package pose_estimator;

import org.junit.jupiter.api.Test;

import config.CameraConfig;
import gtsam.Key;
import gtsam.Matrix;
import gtsam.PlanarProjectionFactor1;
import gtsam.Point2;
import gtsam.Point3;
import gtsam.Pose2;
import gtsam.Vector2;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

public class VisionTest {
    @Test
    void testFar() throws Throwable {
        int lagMicroseconds = 100000;
        Solver solver = new Solver(lagMicroseconds, false);
        CameraConfig conf = new CameraConfig();
        Vision v = new Vision(solver, conf);
        long t_us = 0;
        // put the landmark right in the middle.
        Point3 landmark = new Point3(8, 4, 0.5);
        Point2 measurement = new Point2(400, 300);
        shared_ptr<Diagonal> noise = Diagonal.Sigmas(new Vector2(2, 2));
        shared_ptr<PlanarProjectionFactor1> f = //
                PlanarProjectionFactor1.newPlanarProjectionFactor1(
                        Key.X(t_us),
                        landmark,
                        measurement,
                        conf.camera_offset,
                        conf.calib,
                        noise);
        {
            // Error should be zero.
            System.out.println("landmark is on bore.");
            Matrix H = new Matrix();
            Vector2 err = f.get().evaluateError(new Pose2(4, 4, 0), H);
            System.out.printf("err (%f %f)\n", err.at(0), err.at(1));
            H.print("H");
        }
        {
            // Error should be low.
            System.out.println("landmark is slightly off center");
            Matrix H = new Matrix();
            Vector2 err = f.get().evaluateError(new Pose2(4, 4, 0.1), H);
            System.out.printf("err (%f %f)\n", err.at(0), err.at(1));
            H.print("H");
        }
        {
            // Error should be very high.
            // Note this is probably not high enough, also not all the values are high.
            System.out.println("landmark is behind the camera.");
            Matrix H = new Matrix();
            Vector2 err = f.get().evaluateError(new Pose2(12, 4, 0), H);
            System.out.printf("err (%f %f)\n", err.at(0), err.at(1));
            // Jacobian should be zero.
            H.print("H");
        }
    }

}
