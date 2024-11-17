package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveIOReal implements DriveIO {

  private final CANSparkMax leftMotor =
      new CANSparkMax(DriveConstants.leftMotorID, MotorType.kBrushed);
  private final CANSparkMax leftMotor2 =
      new CANSparkMax(DriveConstants.leftMotor2ID, MotorType.kBrushed);
  private final CANSparkMax rightMotor =
      new CANSparkMax(DriveConstants.rightMotorID, MotorType.kBrushed);
  private final CANSparkMax rightMotor2 =
      new CANSparkMax(DriveConstants.rightMotor2ID, MotorType.kBrushed);
  //  0 is placeholder deviceID
  // private final CANcoder leftEncoder = new CANcoder(0);
  // private final CANcoder rightEncoder = new CANcoder(0);

  private DifferentialDrive driveTrain = new DifferentialDrive(leftMotor, rightMotor);

  // TODO remove cancoders, properly setup pigeon
  public DriveIOReal() {
    leftMotor2.follow(leftMotor);
    rightMotor2.follow(rightMotor);

    CANSparkMax[] motors = {leftMotor, rightMotor, leftMotor2, leftMotor2};

    for (CANSparkMax motor : motors) {
      motor.setIdleMode(IdleMode.kBrake);
    }
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {

    // inputs.leftPosition = leftEncoder.getPosition().getValueAsDouble();

    // inputs.rightPosition = rightEncoder.getPosition().getValueAsDouble();
    // inputs.leftVelocity = leftEncoder.getVelocity().getValueAsDouble();
    // inputs.rightVelocity = rightEncoder.getVelocity().getValueAsDouble();

    inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * 12;
    inputs.leftAppliedVolts2 = leftMotor2.getAppliedOutput() * 12;
    inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * 12;
    inputs.rightAppliedVolts2 = rightMotor2.getAppliedOutput() * 12;
    inputs.leftCurrentAmps = leftMotor.getOutputCurrent();
    inputs.leftCurrentAmps2 = leftMotor2.getOutputCurrent();
    inputs.rightCurrentAmps2 = rightMotor2.getOutputCurrent();
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

  @Override
  public void differentialDrive(double leftSpeed, double rightSpeed) {
    driveTrain.tankDrive(leftSpeed, rightSpeed);
  }
}
