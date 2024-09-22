package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public double motorAppliedVolts = 0.0;
    public double motorCurrentAmps = 0.0;

    public double xSpeed = 0.0;
    public double zRotation = 0.0;
  }

  default void updateInputs(DriveIOInputs inputs) {}

  // stop, run
  default void stopDriveTrain() {}

  default void arcadeDrive(double xSpeed, double omegaRotation) {}
}
