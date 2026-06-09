package util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import gtsam.Pose2;

public class Geometry {
    public static Pose2 toPose2(Pose2d p) throws Throwable {
        return new Pose2(p.getX(), p.getY(), p.getRotation().getRadians());
    }

    public static Pose2d toPose2d(Pose2 p) throws Throwable {
        return new Pose2d(p.x(), p.y(), new Rotation2d(p.theta()));
    }

}
