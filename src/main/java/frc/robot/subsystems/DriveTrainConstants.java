package frc.robot.subsystems;

public interface DriveTrainConstants {

  static final double kWheelRadius = 0.0508;
  static final int kEncoderResolution = 4096;

  // USE CHARACTERIZATION TOOL FOR THIS INFO
  // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization
  public static final double maxVoltage = 7;// 7

  // // this is just an estimate In radians
  // public static final double maxVelocityPerSecond = 7;//2

  // public static final double maxAngularVelocityPerSecond =
  // maxVelocityPerSecond/Math.hypot(0.4041,.4041);
  // when upside down, decreasing offset = turn left/counterclockwise, increasing
  // offset = turn right/clockwise
  // If it's negative increase
  public static final int pigeonID = 10;

  // public static final MechanicalConfiguration mkGearRatio = new
  // MechanicalConfiguration();

  public static final int frontLeftDriveMotor = 1; // FIXME Set front left module drive motor ID
  public static final int frontLeftSteerMotor = 2; // FIXME Set front left module steer motor ID
  public static final int frontLeftSteerEncoder = 52; // FIXME Set front left steer encoder ID
  public static final double frontLeftModuleSteerOffset = Math.toRadians(0); // FIXME Measure and set front left
                                                                             // steer offset

  public static final int frontRightDriveMotor = 3; // FIXME Set front right drive motor ID
  public static final int frontRightSteerMotor = 4; // FIXME Set front right steer motor ID
  public static final int frontRightSteerEncoder = 51; // FIXME Set front right steer encoder ID
  public static final double frontRightModuleSteerOffset = Math.toRadians(0); // FIXME Measure and set front right
  // steer offset

  public static final int backLeftDriveMotor = 5; // FIXME Set back left drive motor ID
  public static final int backLeftSteerMotor = 6; // FIXME Set back left steer motor ID
  public static final int backLeftSteerEncoder = 53; // FIXME Set back left steer encoder ID
  public static final double backLeftModuleSteerOffset = Math.toRadians(90); // FIXME Measure and set
                                                                             // back left
  // steer
  // offset

  public static final int backRightDriveMotor = 7; // FIXME Set back right drive motor ID
  public static final int backRightSteerMotor = 8; // FIXME Set back right steer motor ID
  public static final int backRightSteerEncoder = 50; // FIXME Set back right steer encoder ID
  public static final double backRightModuleSteerOffset = Math.toRadians(45); // FIXME Measure and set back
  // right steer offset

  public static final int volts = 12;
  public static final int voltsSecondsPerMeter = 7;
  public static final int voltSecondsSquaredPerMeter = 5;

  public static final double gearRatio = 5.8 / 1;

}