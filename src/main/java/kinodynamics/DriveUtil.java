package kinodynamics;

import kinodynamics.Odometry.SwerveModuleDelta;
import kinodynamics.Odometry.SwerveModuleDeltas;
import kinodynamics.Odometry.SwerveModulePosition100;
import kinodynamics.Odometry.SwerveModulePositions;

public class DriveUtil {
    public static SwerveModuleDeltas module_position_delta(
            SwerveModulePositions start,
            SwerveModulePositions end) {
        return new SwerveModuleDeltas(
                delta(start.front_left(), end.front_left()),
                delta(start.front_right(), end.front_right()),
                delta(start.rear_left(), end.rear_left()),
                delta(start.rear_right(), end.rear_right()));
    }

    public static SwerveModulePositions module_position_from_delta(
            SwerveModulePositions start, SwerveModuleDeltas delta) {
        return new SwerveModulePositions(
                plus(start.front_left(), delta.front_left()),
                plus(start.front_right(), delta.front_right()),
                plus(start.rear_left(), delta.rear_left()),
                plus(start.rear_right(), delta.rear_right()));
    }

    public static SwerveModuleDelta delta(
            SwerveModulePosition100 start,
            SwerveModulePosition100 end) {
        double delta_m = end.distance() - start.distance();
        return new SwerveModuleDelta(delta_m, end.angle());
    }

    static SwerveModulePosition100 plus(
            SwerveModulePosition100 start,
            SwerveModuleDelta delta) {
        double new_distance_m = start.distance() + delta.distance();
        return new SwerveModulePosition100(new_distance_m, delta.angle());
    }

}
