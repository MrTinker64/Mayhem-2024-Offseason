package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveIOReal implements DriveIO {

  private final CANSparkMax rightMotor =
      new CANSparkMax(DriveConstants.rightMotorID, MotorType.kBrushed);
  private final CANSparkMax leftMotor =
      new CANSparkMax(DriveConstants.leftMotorID, MotorType.kBrushed);

  private DifferentialDrive driveTrain = new DifferentialDrive(leftMotor, rightMotor);

  public void updateInputs(DriveIOInputs inputs) {
    // TODO: Figure out if we will have encoders
  }

  public void stopDriveTrain() {
    driveTrain.stopMotor();
  }

  public void arcadeDrive(double xSpeed, double omegaRotation) {
    driveTrain.arcadeDrive(xSpeed, omegaRotation);
  }
}
