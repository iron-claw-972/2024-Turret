package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    /**
     * The angle of the current, in degrees.
     */
    private double angle;

    private final WPI_TalonFX motor;
    private final CANCoder encoder;
    private final PIDController pid;
    private final SimpleMotorFeedforward feedforward;

    public Turret() {
        // Temporary values
        motor = new WPI_TalonFX(0);
        encoder = new CANCoder(0);
        pid = new PIDController(0, 0, 0);
        feedforward = new SimpleMotorFeedforward(0, 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * Points the turret to the specified angle.
     * <br>
     * Uses soft-stops, stops in the software to ensure the turret doesn't overrotate.
     * @param angle The angle to point to, in degrees.
     *              <br>
     *              This value is reduced to the range [0, 360).
     */
    public void point(double angle) {
        double target = angle % 360;
        double current = encoder.getAbsolutePosition();
    }

    /**
     * Points the turret to the specified angle.
     * <br>
     * Relies on hard-stops to ensure the turret doesn't overrotate.
     * @param angle The angle to point to, in degrees.
     *              <br>
     *              This value is reduced to the range [0, 360).
     */
    public void pointUnsafe(double angle) {

    }

    /**
     * Returns if the angle from the encoder is within
     * the specified tolerance of the target angle.
     * <br>
     * The tolerance is in degrees, and can be found in [todo: insert constants file].
     * @return If the angle is within the tolerance.
     */
    public boolean isAtTarget() {
        return false;
    }

}
