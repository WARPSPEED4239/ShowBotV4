package frc.robot;

public final class Constants {
    public static final int DRIVETRAIN_MOTOR_LEFT_1 = 1, //SPARK MAX
                            DRIVETRAIN_MOTOR_LEFT_2 = 2,
                            DRIVETRAIN_MOTOR_RIGHT_1 = 4,
                            DRIVETRAIN_MOTOR_RIGHT_2 = 5,

                            CANNON_ANGLE_ADJUST_MOTOR = 3,
                            CANNON_REVOLVE_MOTOR = 6,
                            
                            CANIFIER = 1;

    public static final int CANNON_REVOLVE_LIMIT_SWITCH = 0; //DIO

    public static final int CANNON_STORAGE_PRESSURE = 0, //ANALOG IN
                            CANNON_FIRING_PRESSURE = 1;

    public static final int CANNON_LOADING_SOLENOID = 0, //PCM
                            CANNON_FIRING_SOLENOID = 3;

    public static final double EPSILON = 0.0001,
                               MIN_FIRING_PRESSURE = 80.0,
                               MAX_FIRING_PRESSURE = 85.0,
                               CANNON_ROTATION_SPEED = 0.5;
}
