package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurretConstants;

/**
 * The Turret subsystem. All measurements in degrees.
 */
@SubsystemImpl(TurretImpl.class)
public class Turret extends SubsystemBase {

    // Simulation
    private Mechanism2d simulationMechanism;
    private MechanismLigament2d simulationLigament;

    double angle;
    double targetAngle;

    public Turret() {
        if (RobotBase.isSimulation()) {
            simulationMechanism = new Mechanism2d(3, 3);
            MechanismRoot2d mechanismRoot = simulationMechanism.getRoot("Turret", 1.5, 1.5);
            simulationLigament = mechanismRoot.append(
                    new MechanismLigament2d("angle", 1, 0, 4, new Color8Bit(Color.kOrange))
            );

            SmartDashboard.putData("Turret", simulationMechanism);
        }
    }

    @Override
    public void periodic() {
        double add = (targetAngle - angle) * 0.2;
        angle += add;
    }

    @Override
    public void simulationPeriodic() {
        simulationLigament.setAngle(angle);
    }

    /**
     * Points the turret to the specified angle.
     * <br>
     * Uses soft-stops, stops in the software to
     * ensure the turret doesn't overrotate.
     * <br>
     * Methods that override this method should
     * call the super method.
     * @param angle The angle to point to, in degrees.
     *              <br>
     *              This value is reduced to the range [0, 360).
     */
    public void point(double angle) {
        // TODO: make [-180, 180] instead of [0, 360)
        angle %= 360;
        this.targetAngle = angle;
    }

    /**
     * Returns if the angle from the encoder is within
     * the specified tolerance of the target angle.
     * <br>
     * The tolerance is in degrees, and
     * can be found in {@link TurretConstants}.
     *
     * @return If the angle is within the tolerance.
     */
    public boolean isAtTarget() {
        return Math.abs(angle - targetAngle) < TurretConstants.TOLERANCE;
    }

    /**
     * @return Returns the current angle of the turret.
     */
    public double getAngle() {
        return angle;
    }

}
