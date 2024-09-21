package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class PoseManager {

  private Rotation2d lastGyroAngle = new Rotation2d();
  private Twist2d robotVelocity = new Twist2d();
  private double lastYawVelocity = 0.0;
  private double lastLeftPosition = 0.0;
  private double lastRightPosition = 0.0;

  // DifferentialDrivePoseEstimator​(DifferentialDriveKinematics kinematics, Rotation2d gyroAngle,
  // double leftDistanceMeters, double rightDistanceMeters, Pose2d initialPoseMeters)
  private DifferentialDrivePoseEstimator poseEstimator =
      new DifferentialDrivePoseEstimator(
          DriveConstants.kinematics,
          lastGyroAngle,
          lastLeftPosition,
          lastRightPosition,
          new Pose2d());

  public PoseManager() {}

  public void addOdometryMeasurement(
      Rotation2d gyroAngle, double leftPosition, double rightPosition, double yawVelocity) {
    lastGyroAngle = gyroAngle;
    // might need to convert to distance
    poseEstimator.update(gyroAngle, leftPosition, rightPosition);
    lastYawVelocity = yawVelocity;
    lastLeftPosition = leftPosition;
    lastRightPosition = rightPosition;
  }

  public void addVisionMeasurement(
      Pose2d estimatedPose, double timestamp, Matrix<N3, N1> stdDevs, int tagCount) {
    // Exit if the robot's angular velocity is too high
    if (Math.abs(lastYawVelocity) > 720) {
      return;
    }
    // Add result because all checks passed
    poseEstimator.addVisionMeasurement(estimatedPose, timestamp, stdDevs);
  }

  public void addVelocityData(Twist2d robotVelocity) {
    this.robotVelocity = robotVelocity;
  }

  public double getDistanceTo(Pose2d pose) {
    return getDistanceTo(pose.getTranslation());
  }

  public double getDistanceTo(Translation3d translation) {
    return getDistanceTo(translation.toTranslation2d());
  }

  public double getDistanceTo(Translation2d translation) {
    Translation2d currentTranslation = getPose().getTranslation();
    return currentTranslation.getDistance(translation);
  }

  public Rotation2d getHorizontalAngleTo(Pose2d pose) {
    return getHorizontalAngleTo(pose.getTranslation());
  }

  public Rotation2d getHorizontalAngleTo(Translation3d translation) {
    return getHorizontalAngleTo(translation.toTranslation2d());
  }

  public Rotation2d getHorizontalAngleTo(Translation2d translation) {
    Translation2d currentTranslation = getPose().getTranslation();
    Rotation2d theta = currentTranslation.minus(translation).getAngle();
    return theta;
  }

  public Rotation2d getVerticalAngleTo(Translation3d translation) {
    double horizontalDiff = getDistanceTo(translation);
    double zDiff = translation.getZ();
    Rotation2d theta = new Rotation2d(Math.atan2(zDiff, horizontalDiff));
    return theta;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the current odometry translation. */
  public Translation2d getTranslation() {
    return getPose().getTranslation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(lastGyroAngle, lastLeftPosition, lastRightPosition, pose);
  }

  @AutoLogOutput(key = "Odometry/FieldVelocity")
  public Twist2d fieldVelocity() {
    Translation2d linearFieldVelocity =
        new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(getPose().getRotation());
    return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
  }
}
