package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import frc.robot.constants.Constants;
import frc.robot.constants.TurretConstants;
import frc.robot.util.ConversionUtils;
import frc.robot.util.MathUtils;
import frc.robot.util.MotorFactory;

public class Turret extends SubsystemBase {

    private final WPI_TalonFX motor;
    private final DutyCycleEncoder encoder;
    private final PIDController pid;

    // Simulation
    private DutyCycleEncoderSim encoderSim;
    private Mechanism2d simulationMechanism;
    private MechanismLigament2d simulationLigament;
    private double simulationAngle;

    private final PIDController visionPid;

    public Turret() {
        // Temporary values
        motor = MotorFactory.createTalonFXSupplyLimit(
                TurretConstants.MOTOR_ID,
                Constants.kRioCAN,
                TurretConstants.CONTINUOUS_CURRENT_LIMIT,
                TurretConstants.PEAK_CURRENT_LIMIT,
                TurretConstants.PEAK_CURRENT_DURATION
        );
        motor.setNeutralMode(TurretConstants.NEUTRAL_MODE);
        motor.setInverted(TurretConstants.INVERTED);
        motor.enableVoltageCompensation(true);

        encoder = new DutyCycleEncoder(TurretConstants.ENCODER_PORT);
        encoder.setDistancePerRotation(360);

        pid = new PIDController(
                TurretConstants.P,
                TurretConstants.I,
                TurretConstants.D
        );
        pid.setTolerance(TurretConstants.TOLERANCE);

        visionPid = new PIDController(
                0,
                0,
                0
        );

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
    public void periodic() {
        double current = encoder.getAbsolutePosition();
        double power = pid.calculate(current, pid.getSetpoint());
        setMotorPower(power);
    }

    @Override
    public void simulationPeriodic() {
        double toTurn = simulationAngle * 0.1;
        if (Math.abs(toTurn) < 0.005) {
            simulationAngle = 0;
            return;
        }
        simulationAngle -= toTurn;
        simulationLigament.setAngle(simulationLigament.getAngle() + toTurn);
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
        angle %= 360;
        double currentAngle = encoder.getAbsolutePosition() - encoder.getPositionOffset();
        double toTurn = angle - currentAngle;

        setSetpoint(pid.getSetpoint() + toTurn);

        if (RobotBase.isSimulation()) {
            simulationAngle = angle;
        }
    }

    protected void setSetpoint(double setpoint) {
        pid.reset();
        pid.setSetpoint(setpoint);
    }

    protected void setMotorPower(double power) {
        power = MathUtil.clamp(power, -TurretConstants.MOTOR_POWER_CLAMP, TurretConstants.MOTOR_POWER_CLAMP);
        motor.set(power);
    }

    /**
     * Returns if the angle from the encoder is within
     * the specified tolerance of the target angle.
     * <br>
     * The tolerance is in degrees, and can be found in [todo: insert constants file].
     *
     * @return If the angle is within the tolerance.
     */
    public boolean isAtTarget() {
        return pid.atSetpoint();
    }

}
