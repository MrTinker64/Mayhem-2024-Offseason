package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public double leftPosition = 0.0;
    public double rightPosition = 0.0;

    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double rightCurrentAmps = 0.0;
  }

  default void updateInputs(DriveIOInputs inputs) {}
  // default void stopDriveTrain() {}
  default void arcadeDrive(double xSpeed, double omegaRotation) {}
}
