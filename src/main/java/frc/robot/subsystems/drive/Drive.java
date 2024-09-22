package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final DriveIOReal io;
  private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();

  public Drive(DriveIOReal io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(driveInputs);
    Logger.processInputs("Shooter/Feeder", driveInputs);
    GeneralUtil.logSubsystem(this, "Shooter/Feeder");
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
