package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;
import frc.robot.util.MotorFactory;

public class TurretImpl extends Turret {

    private final WPI_TalonFX motor;
    private final PIDController pid;

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

        pid = new PIDController(
                TurretConstants.P,
                TurretConstants.I,
                TurretConstants.D
        );
        pid.setTolerance(TurretConstants.TOLERANCE);

    }

    @Override
    public void periodic() {
        double current = encoder.getAbsolutePosition();
        double power = pid.calculate(current, pid.getSetpoint());
        setMotorPower(power);
    }

    @Override
    public double point(double angle) {
        double toTurn = super.point(angle);
        setSetpoint(pid.getSetpoint() + toTurn);
        return toTurn;
    }

    protected void setSetpoint(double setpoint) {
        pid.reset();
        pid.setSetpoint(setpoint);
    }

    protected void setMotorPower(double power) {
        power = MathUtil.clamp(power, -TurretConstants.MOTOR_POWER_CLAMP, TurretConstants.MOTOR_POWER_CLAMP);
        motor.set(power);
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
