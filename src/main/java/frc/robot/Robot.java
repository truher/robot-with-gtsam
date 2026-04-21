package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    private final Sim sim;

    public Robot() {
        System.out.println("Robot()");
        String cwd = new File("").getAbsolutePath();
        System.out.println("*************************************************************************************");
        System.out.printf("CWD %s\n", cwd);
        System.out.println("*************************************************************************************");
        sim = Sim.make();
        // run once
        try {
            sim.run();
        } catch (Throwable e) {
            System.out.println("*************************************************************************************");
            e.printStackTrace();
        }
        // sim=null;
    }

    @Override
    public void teleopPeriodic() {
        System.out.println("*************************************************************************************");
        System.out.println("teleopPeriodic()");
        if (sim != null) {
            try {
                sim.run();
            } catch (Throwable e) {
                System.out.println("*************************************************************************************");
                e.printStackTrace();
            }
        } else {
            System.out.println("*************************************************************************************");
            System.out.println("sim is null");
        }
        System.out.flush();
    }
}
