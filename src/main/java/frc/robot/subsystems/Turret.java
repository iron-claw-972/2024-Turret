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
 * Simulation for the turret subsystem for robots that don't have a turret.
 */
@SubsystemImpl(TurretImpl.class)
public class Turret extends SubsystemBase {

    private MechanismLigament2d simulationLigament;

    /**
     * The current angle of the turret in degrees.
     * <br>
     * Used for simulation.
     */
    double angle;

    /**
     * The target angle of the turret in degrees.
     * <br>
     * Used for simulation.
     */
    double targetAngle;

    public Turret() {
        // This turret implementation will only be used for robots that don't have a turret, both real and
        // simulated. Because of this, the isSimulation() check is needed in order to not expend resources to an
        // unneeded task in the case of a real robot.
        if (RobotBase.isSimulation()) {
            Mechanism2d simulationMechanism = new Mechanism2d(3, 3);
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
     * Points the turret to the specified angle in degrees.
     * <br>
     * Uses soft-stops, stops in the software to ensure the turret doesn't overrotate.
     * @param angle The angle to point to in degrees.
     *              <br>
     *              This value is reduced to the range [0, 360).
     */
    public void point(double angle) {
        
        this.targetAngle = parseAngle(angle);
    }

    /**
     * Returns if the angle from the encoder is within the specified tolerance of the target angle.
     * <br>
     * The tolerance is in degrees, and can be found in {@link TurretConstants}.
     * @return If the angle is within the tolerance.
     */
    public boolean isAtTarget() {
        return Math.abs(angle - targetAngle) < TurretConstants.TOLERANCE;
    }

    /**
     * @return The current angle of the turret in degrees.
     */
    public double getAngle() {
        return angle;
    }


    /**
     * @return parsed angle between [-180, 180]
     */
    protected double parseAngle(double angle){
        angle %= 360;

        if (angle > 180) {
            angle -= 360;
        } 

        if (angle < -180) {
            angle += 360;
        } 

        return angle;

        
    }

}
