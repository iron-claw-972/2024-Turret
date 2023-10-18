package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

import java.util.List;

/**
 * Set of known Robot Names.
 * <p>The name of a robot in the RoboRIO's persistent memory.
 * At deploy time, that name is used to set the corresponding RobotId.
 * <p>Note that the RobotId is determined at Deploy time.
 */
public enum RobotId {
    Default,
    TurretTest(Turret.class, Drivetrain.class),
    SwerveCompetition, SwerveTest,
    ClassBot1, ClassBot2, ClassBot3, ClassBot4;

    private final List<Class<? extends SubsystemBase>> subsystems;

    @SafeVarargs
    RobotId(Class<? extends SubsystemBase>... subsystems) {
        this.subsystems = List.of(subsystems);
    }

    public List<Class<? extends SubsystemBase>> getSubsystems() {
        return subsystems;
    }

    public boolean isClassBot() {
        return this == ClassBot1 || this == ClassBot2 || this == ClassBot3 || this == ClassBot4;
    }

    public boolean isSwerveBot() {
        return this == SwerveCompetition || this == SwerveTest;
    }

}
