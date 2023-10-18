package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;
import frc.robot.util.MotorFactory;

public class TurretImpl extends Turret {

    private final DutyCycleEncoder encoder;
    private final WPI_TalonFX motor;
    private final PIDController pid = new PIDController(
            TurretConstants.P,
            TurretConstants.I,
            TurretConstants.D
    );
    private final PIDController visionPid = new PIDController(
            0,
            0,
            0
    );

    public TurretImpl() {

        System.out.println("TurretImpl");

        motor = MotorFactory.createTalonFXSupplyLimit(
                TurretConstants.MOTOR_ID,
                Constants.RIO_CAN,
                TurretConstants.CONTINUOUS_CURRENT_LIMIT,
                TurretConstants.PEAK_CURRENT_LIMIT,
                TurretConstants.PEAK_CURRENT_DURATION
        );
        motor.setNeutralMode(TurretConstants.NEUTRAL_MODE);
        motor.setInverted(TurretConstants.INVERTED);
        motor.enableVoltageCompensation(true);

        encoder = new DutyCycleEncoder(TurretConstants.ENCODER_PORT);
        encoder.setDistancePerRotation(360);

        pid.setTolerance(TurretConstants.TOLERANCE);

        // TODO: Constants
        visionPid.setTolerance(0.1);

    }

    @Override
    public void periodic() {
        double current = encoder.getDistance();
        double power = pid.calculate(current);

        motor.set(MathUtil.clamp(power, -TurretConstants.MOTOR_POWER_CLAMP, TurretConstants.MOTOR_POWER_CLAMP));
    }

    @Override
    public void simulationPeriodic() {
        // TODO: Update encoder sim position
//        motor.getSimCollection()
        // TODO: FIX
    }

    @Override
    public void point(double angle) {
        pid.reset();
        pid.setSetpoint(angle);
    }

    @Override
    public boolean isAtTarget() {
        return pid.atSetpoint();
    }

    @Override
    public double getAngle() {
        return encoder.getAbsolutePosition();
    }

}
