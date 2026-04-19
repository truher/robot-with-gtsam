package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import gtsam.Point2;

public class Robot extends TimedRobot {

    public Robot() {
    }

    @Override
    public void teleopPeriodic() {
        try {
            Point2 p = new Point2(4, 5);
            p.print();
        } catch (Throwable e) {
            e.printStackTrace();
        }
    }
}
