package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    private final Sim sim;

    public Robot() {
        DataLogManager.start();
        System.out.println("Robot()");
        String cwd = new File("").getAbsolutePath();
        System.out.printf("CWD %s\n", cwd);
        sim = Sim.make();
        // run once
        try {
            if (sim == null) return; 
            sim.run();
        } catch (Throwable e) {
            e.printStackTrace();
        }
        // sim=null;
    }

    @Override
    public void teleopPeriodic() {
        if (sim != null) {
            try {
                sim.run();
            } catch (Throwable e) {
                e.printStackTrace();
            }
        } else {
            System.out.println("sim is null");
        }
        System.out.flush();
    }
}
