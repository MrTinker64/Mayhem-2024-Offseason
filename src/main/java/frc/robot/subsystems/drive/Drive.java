package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import frc.robot.util.PoseManager;
import frc.robot.util.loggedShuffleboardClasses.LoggedShuffleboardNumber;
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

  private Timer autoTimer = new Timer();
  private LoggedShuffleboardNumber forwardDuration =
      new LoggedShuffleboardNumber("forward", "Test", 2.8);
  private LoggedShuffleboardNumber backDuration = new LoggedShuffleboardNumber("back", "Test", .7);

  public SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

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
        new ReplanningConfig(),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    io.updateInputs(driveInputs);
    Logger.processInputs("Drive/Inputs", driveInputs);
    GeneralUtil.logSubsystem(this, "Drive");

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
          io.arcadeDrive(
              Math.copySign(xSpeed * xSpeed, xSpeed),
              Math.copySign(omegaRotation * omegaRotation, omegaRotation));
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

    // TODO:pass in parameters
    // setVelocity();

  }

  public void setVelocity(double leftSpeedMetersPerSecond, double rightSpeedMetersPerSecond) {
    // Calculate the voltage needed for each side
    double leftFeedforward = feedforward.calculate(leftSpeedMetersPerSecond);
    double rightFeedforward = feedforward.calculate(rightSpeedMetersPerSecond);

    // Set the motor voltages with feedforward applied
    io.differentialDrive(leftFeedforward, rightFeedforward);
  }

  public Command auto() {
    double volts = 3;
    return runEnd(() -> io.voltageDrive(volts, volts), () -> autoTimer.stop())
        .until(() -> autoTimer.get() >= 1.7)
        .beforeStarting(
            () -> {
              autoTimer.restart();
              autoTimer.start();
            });
  }

  public Command autoWithCube() {
    Command pushBack =
        runEnd(() -> io.voltageDrive(-3, -3), () -> autoTimer.stop())
            .until(() -> autoTimer.get() >= backDuration.get(() -> true))
            .beforeStarting(
                () -> {
                  autoTimer.restart();
                  autoTimer.start();
                });
    Command goForwards =
        runEnd(() -> io.voltageDrive(3, 3), () -> autoTimer.stop())
            .until(() -> autoTimer.get() >= forwardDuration.get(() -> true))
            .beforeStarting(
                () -> {
                  autoTimer.restart();
                  autoTimer.start();
                });
    return pushBack.andThen(goForwards);
  }
}
