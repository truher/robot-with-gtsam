package frc.robot;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import gtsam.Pose2;
import gtsam.Vector;
import gtsam.Vector3;
import util.Geometry;

public class SimTest {
    /** This should see chirality exceptions for awhile, and then stop.  */
    @Test
    void testSim() throws Throwable {
        Sim sim = new Sim();
        for (int i = 0; i < 100; ++i) {
            sim.run();
            Pose2d gt = sim.groundTruthPose();
            Pose2 gtp2 = Geometry.toPose2(gt);
            Pose2 p = sim.estimatedPose();
            Vector3 err = gtp2.local(p);
            System.out.printf("err (%f %f %f)\n", err.at(0), err.at(1), err.at(2));
            Vector s = sim.poseSigma();
            System.out.printf("%d gt (%f %f %f) est (%f %f %f) +/- (%f %f %f\n",
                    i, gt.getX(), gt.getY(), gt.getRotation().getRadians(),
                    p.x(), p.y(), p.theta(), s.at(0), s.at(1), s.at(2));
        }
    }
}
