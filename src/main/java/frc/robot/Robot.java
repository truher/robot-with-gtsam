package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    private final Sim sim;

    public Robot() {
        sim = Sim.make();
    }

    @Override
    public void teleopPeriodic() {
        if (sim != null) {
            try {
                sim.run();
            } catch (Throwable e) {
                e.printStackTrace();
            }
        }
    }
}
