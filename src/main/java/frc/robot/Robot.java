package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    private final Sim sim;

    public Robot() {
        DataLogManager.start();
        // After experimenting awhile, I think the incremental smoother
        // is not worth pursuring for 2027.
        sim = new Sim(false);
    }

    @Override
    public void teleopPeriodic() {
        sim.run();
    }
}
