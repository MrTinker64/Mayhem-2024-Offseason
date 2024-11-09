package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public double leftPosition = 0.0;
    public double rightPosition = 0.0;
    public double leftVelocity = 0.0;
    public double rightVelocity = 0.0;

    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double rightCurrentAmps = 0.0;
    
     SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);
  }

  default void updateInputs(DriveIOInputs inputs) {}

  default void arcadeDrive(double xSpeed, double omegaRotation) {}
  default void setVelocity(){};
  default void differentialDrive(double leftSpeed, double rightSpeed){};
 
}
