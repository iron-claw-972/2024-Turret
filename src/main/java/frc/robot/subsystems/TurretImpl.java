package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotId;
import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;
import frc.robot.util.MotorFactory;

/**
 * Motor-based implementation of the turret subsystem. Only created for robots marked with having a turret in
 * {@link RobotId}.
 */
public class TurretImpl extends Turret {

    private final DutyCycleEncoder encoder;
    private final WPI_TalonFX motor;
    private final PIDController pid = new PIDController(
            TurretConstants.P,
            TurretConstants.I,
            TurretConstants.D
    );

    private SingleJointedArmSim sim;
    private DutyCycleEncoderSim encoderSim;

    public TurretImpl() {
        super();

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

        if (RobotBase.isSimulation()) {
            sim = new SingleJointedArmSim(
                    DCMotor.getFalcon500(1),
                    TurretConstants.GEAR_RATIO,
                    TurretConstants.MOMENT_OF_INERTIA,
                    TurretConstants.RADIUS,
                    Math.toRadians(-180),
                    Math.toRadians(180),
                    false
            );
            encoderSim = new DutyCycleEncoderSim(encoder);
        }

        pid.setTolerance(TurretConstants.TOLERANCE);

    }

    @Override
    public void periodic() {
        double current = encoder.getDistance();
        double power = pid.calculate(current);

        System.out.println("Power: " + power + " Current: " + current + " Setpoint: " + pid.getSetpoint() + " Error: " + pid.getPositionError() + " At Setpoint: " + pid.atSetpoint());

        motor.set(MathUtil.clamp(power, -TurretConstants.MOTOR_POWER_CLAMP,
                TurretConstants.MOTOR_POWER_CLAMP));
    }

    @Override
    public void simulationPeriodic() {

        motor.getSimCollection().setBusVoltage(RobotController.getBatteryVoltage());

        sim.setInput(motor.getSimCollection().getMotorOutputLeadVoltage() * RobotController.getBatteryVoltage());
        sim.update(Constants.LOOP_TIME);

//        System.out.println(RobotController.getBatteryVoltage());

        encoderSim.setDistance(Math.toDegrees(sim.getAngleRads()));
        simulationLigament.setAngle(Math.toDegrees(sim.getAngleRads()));
    }

    @Override
    public void point(double angle) {
        angle %= 360;
        pid.reset();
        pid.setSetpoint(angle);
        System.out.println("Setpoint: " + angle);
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
