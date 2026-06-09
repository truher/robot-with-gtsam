package util;

import edu.wpi.first.math.geometry.Pose2d;
import gtsam.Pose2;

public class Geometry {
    public static Pose2 toPose2(Pose2d p) throws Throwable {
        return new Pose2(p.getX(), p.getY(), p.getRotation().getRadians());
    }

}
