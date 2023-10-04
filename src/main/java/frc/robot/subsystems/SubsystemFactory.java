package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotId;

public class SubsystemFactory {

    public static SubsystemBase get(Class<SubsystemBase> clazz) {
        RobotId robotId = Robot.getRobotId();
        return RobotBase.isReal() ?
                SUBSYSTEMS.get(clazz).getFirst() : SUBSYSTEMS.get(clazz).getSecond();
    }

    // Jerry wants a system in which you can run the robot's code on any other robot and itll launch and run
    // properly, except the simulation version of nonexisting subsytems would be created how?


}
