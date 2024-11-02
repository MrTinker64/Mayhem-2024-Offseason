package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import frc.robot.util.PoseManager;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final DriveIO io;
  private final GyroIO gyroIO;
  private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private PoseManager poseManager;
  private static final double DEADBAND = 0.05;

  Rotation2d rawGyroRotation = new Rotation2d();

  public Drive(DriveIO io, GyroIO gyroIO, PoseManager poseManager) {
    this.io = io;
    this.gyroIO = gyroIO;
    this.poseManager = poseManager;

    AutoBuilder.configureRamsete(
        () -> poseManager.getPose(),
        (pose) -> {
          poseManager.setPose(pose);
        },
        () -> getSpeeds(),
        (speeds) -> {
          var wheelSpeeds = DriveConstants.kinematics.toWheelSpeeds(speeds);
          driveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
        },
        DEADBAND,
        DEADBAND,
        null,
        null,
        null);
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    io.updateInputs(driveInputs);
    Logger.processInputs("Drive/Inputs", driveInputs);
    GeneralUtil.logSubsystem(this, "Drive/Inputs");

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist =
          DriveConstants.kinematics.toTwist2d(driveInputs.leftPosition, driveInputs.rightPosition);
      rawGyroRotation = new Rotation2d(twist.dtheta);
    }

    // Add odometry measurement
    poseManager.addOdometryMeasurement(
        rawGyroRotation,
        driveInputs.leftPosition,
        driveInputs.rightPosition,
        gyroInputs.yawVelocityRadPerSec);
  }

  public Command joystickDrive(Supplier<Double> xInput, Supplier<Double> omegaRotationInput) {
    return run(() -> {
          double xSpeed = MathUtil.applyDeadband(xInput.get(), DEADBAND);
          double omegaRotation = MathUtil.applyDeadband(omegaRotationInput.get(), DEADBAND);
          io.arcadeDrive(xSpeed, omegaRotation);
        })
        .withName("joystick drive");
  }

  /** Returns the measured speeds of the robot in the robot's frame of reference. */
  @AutoLogOutput(key = "Drive/MeasuredSpeeds")
  private ChassisSpeeds getSpeeds() {
    return DriveConstants.kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(driveInputs.leftVelocity, driveInputs.rightVelocity));
  }

  public void driveVelocity(double leftMetersPerSec, double rightMetersPerSec) {
    Logger.recordOutput("Drive/LeftVelocitySetpointMetersPerSec", leftMetersPerSec);
    Logger.recordOutput("Drive/RightVelocitySetpointMetersPerSec", rightMetersPerSec);
    double leftRadPerSec = leftMetersPerSec / DriveConstants.WHEEL_RADIUS;
    double rightRadPerSec = rightMetersPerSec / DriveConstants.WHEEL_RADIUS;
    // TODO: implement io.setVelocity
    // io.setVelocity(
    //     leftRadPerSec,
    //     rightRadPerSec,
    //     feedforward.calculate(leftRadPerSec),
    //     feedforward.calculate(rightRadPerSec));
  }
}
