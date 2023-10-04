package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotId;

public class SubsystemFactory {

    public static SubsystemBase get(Class<?> clazz) {
        RobotId robotId = Robot.getRobotId();
        try {
            for (Class<? extends SubsystemBase> subsystem : robotId.getSubsystems()) {
                if (subsystem.equals(clazz)) {
                    try {
                        if (RobotBase.isReal()) {
                            Class<? extends SubsystemBase> impl = subsystem.getAnnotation(SubsystemImpl.class).value();
                            return impl.getDeclaredConstructor().newInstance();
                        }
                        return subsystem.getDeclaredConstructor().newInstance();
                    } catch (Exception e) {
                        DriverStation.reportError("Could not create subsystem " + clazz.getSimpleName(), e.getStackTrace());
                    }
                }
            }
            return (SubsystemBase) clazz.getDeclaredConstructor().newInstance();
        } catch (Exception e) {
            DriverStation.reportError("Could not create subsystem " + clazz.getSimpleName(), e.getStackTrace());
        }
        return null;
    }

}
