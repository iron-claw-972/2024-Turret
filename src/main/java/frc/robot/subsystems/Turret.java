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

    public void point(double angle) {
        double target = angle % 360;
        double current = encoder.getAbsolutePosition();
    }




}
