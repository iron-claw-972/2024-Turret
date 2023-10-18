package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Container class for turret constants.
 */
public class TurretConstants {

    private TurretConstants() {}

    public static final int MOTOR_ID = 0;

    public static final int GEAR_RATIO = 1;
    public static final int MOMENT_OF_INERTIA = 1;
    public static final double RADIUS = 1;

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