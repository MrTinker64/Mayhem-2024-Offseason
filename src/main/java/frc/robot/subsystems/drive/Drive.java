package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final DriveIO io;
  private final GyroIO gyroIO;
  private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  Rotation2d rawGyroRotation = new Rotation2d();

  public Drive(DriveIO io, GyroIO gyroIO) {
    this.io = io;
    this.gyroIO = gyroIO;
  }

  public void periodic() {
    io.updateInputs(driveInputs);
    Logger.processInputs("Shooter/Feeder", driveInputs);
    GeneralUtil.logSubsystem(this, "Shooter/Feeder");

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist =
          DriveConstants.kinematics.toTwist2d(driveInputs.leftPosition, driveInputs.rightPosition);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }
  }

  private void fullStop() {
    io.stopDriveTrain();
  }

  private void driveForwardFullSpeed() {
    io.arcadeDrive(1.0, 0);
  }

  private void arcadeDrive(double speed, double omegaRotation) {
    io.arcadeDrive(speed, omegaRotation);
  }

  public Command joystickDrive(Supplier<Double> xInput, Supplier<Double> omegaRotationInput) {
    return run(() -> {
          double xSpeed = xInput.get();

          if (xSpeed == 0) {
            fullStop();
          }

          double omegaRotation = omegaRotationInput.get();
          arcadeDrive(xSpeed, omegaRotation);
        })
        .withName("joystick drive");
  }
}
