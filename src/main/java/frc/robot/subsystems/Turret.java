package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurretConstants;

public class Turret extends SubsystemBase {

    final DutyCycleEncoder encoder;

    // Simulation
    private DutyCycleEncoderSim encoderSim;
    private Mechanism2d simulationMechanism;
    private MechanismLigament2d simulationLigament;
    private double simulationAngle;

    private final PIDController visionPid;

    public Turret() {

        encoder = new DutyCycleEncoder(TurretConstants.ENCODER_PORT);

        visionPid = new PIDController(
                0,
                0,
                0
        );
        // TODO: Constant
        visionPid.setTolerance(0.1);

        if (RobotBase.isSimulation()) {

            encoderSim = new DutyCycleEncoderSim(encoder);

            simulationMechanism = new Mechanism2d(3, 3);
            MechanismRoot2d mechanismRoot = simulationMechanism.getRoot("Turret", 1.5, 1.5);
            simulationLigament = mechanismRoot.append(
                    new MechanismLigament2d("angle", 1, 0, 4, new Color8Bit(Color.kOrange))
            );

            SmartDashboard.putData("Turret", simulationMechanism);
        }
    }

    @Override
    public void simulationPeriodic() {
        double toTurn = simulationAngle * 0.1;
        // TODO: We probably need to make a limit to how fast the turret can turn
        if (Math.abs(toTurn) < 0.001) {
            simulationAngle = 0;
            return;
        }
        simulationAngle -= toTurn;
        simulationLigament.setAngle(simulationLigament.getAngle() + toTurn);
        encoderSim.set(simulationLigament.getAngle());
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
    public double point(double angle) {
        angle %= 360;
        double currentAngle = getAngle();
        double toTurn = angle - currentAngle;

        simulationAngle += toTurn;
        return toTurn;
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
        return simulationAngle == 0;
    }

    /**
     * @return Returns the current angle of the turret.
     */
    public double getAngle() {
        return encoder.get();
    }

}
