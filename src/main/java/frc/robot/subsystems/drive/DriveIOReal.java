package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveIOReal implements DriveIO {

  private final CANSparkMax leftMotor =
  new CANSparkMax(DriveConstants.leftMotorID, MotorType.kBrushed);
  private final CANSparkMax rightMotor =
      new CANSparkMax(DriveConstants.rightMotorID, MotorType.kBrushed);
  // 0 is placeholder deviceID
  private final CANcoder leftEncoder = new CANcoder(0);
  private final CANcoder rightEncoder = new CANcoder(0);

  private DifferentialDrive driveTrain = new DifferentialDrive(leftMotor, rightMotor);

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPosition = leftEncoder.getPosition().getValueAsDouble();
    inputs.rightPosition = rightEncoder.getPosition().getValueAsDouble();

    inputs.leftAppliedVolts = leftMotor.getAppliedOutput()*12;
    inputs.rightAppliedVolts = rightMotor.getAppliedOutput()*12;
    inputs.leftCurrentAmps = leftMotor.getOutputCurrent();
    inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
  }

  @Override
  public void arcadeDrive(double xSpeed, double omegaRotation) {
    if (xSpeed == 0 && omegaRotation == 0) {
      driveTrain.stopMotor();
    } else {
      driveTrain.arcadeDrive(xSpeed, omegaRotation);
    }
  }
}
