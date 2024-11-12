package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class DriveConstants {
  // TODO: Change these ids
  public static final int rightMotorID = 0;
  public static final int leftMotorID = 0;

  public static final int WHEEL_RADIUS = 3; //  wheel radius is 3 inches

  // TODO: FAKE change trackwidth later
  public static final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(0.508);

  // TODO: Tune THese
  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kA = 0;
}
