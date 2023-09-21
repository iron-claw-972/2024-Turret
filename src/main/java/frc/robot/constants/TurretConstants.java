package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot.RobotId;
import frc.robot.subsystems.Turret;

import java.util.ArrayList;
import java.util.List;

/**
 * Container class for turret constants.
 */
public class TurretConstants {

    private TurretConstants() {}

    public static final int MOTOR_ID = 0;

    public static final int CONTINUOUS_CURRENT_LIMIT = 40;
    public static final int PEAK_CURRENT_LIMIT = 60;
    public static final int PEAK_CURRENT_DURATION = 100;

    public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
    public static final boolean INVERTED = false;

    public static final int ENCODER_PORT = 0;

    public static final double P = 0;
    public static final double I = 0;
    public static final double D = 0;

    public static final double MOTOR_POWER_CLAMP = 0.5;

    public static final double TOLERANCE = 0;

}