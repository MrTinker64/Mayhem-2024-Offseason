package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class DriveIOSim implements DriveIO {

  private DifferentialDrivetrainSim driveTrain =
      new DifferentialDrivetrainSim(
          DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
          7.29, // 7.29:1 gearing reduction.
          7.5, // MOI of 7.5 kg m^2 (from CAD model).
          60.0, // The mass of the robot is 60 kg.
          Units.inchesToMeters(3), // The robot uses 3" radius wheels.
          0.7112, // The track width is 0.7112 meters.

          // The standard deviations for measurement noise:
          // x and y:          0.001 m
          // heading:          0.001 rad
          // l and r velocity: 0.1   m/s
          // l and r position: 0.005 m
          VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

 
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);

  private static final double kTrackWidth = 0.381 * 2;

  private final PWMSparkMax m_leftLeader = new PWMSparkMax(1);
  private final PWMSparkMax m_leftFollower = new PWMSparkMax(2);
  private final PWMSparkMax m_rightLeader = new PWMSparkMax(3);
  private final PWMSparkMax m_rightFollower = new PWMSparkMax(4);

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);

  public DriveIOSim() {
    m_leftLeader.addFollower(m_leftFollower);
    m_rightLeader.addFollower(m_rightFollower);
  }
   @Override
  public void updateInputs(DriveIOInputs inputs) {
    // TODO: we will have encoders

  }

  @Override
  public void stopDriveTrain() {
    driveTrain.setInputs(0, 0);
  }


  @Override
  public void arcadeDrive(double xSpeed, double omegaRotation) {

    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, omegaRotation)));
  }

  /** Sets speeds to the drivetrain motors. */
  //TODO there is a unit's conflict
  private void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward =
        m_feedforward.calculate(MetersPerSecond.of(speeds.leftMetersPerSecond)).in(Volts);
    final double rightFeedforward =
        m_feedforward.calculate(MetersPerSecond.of(speeds.rightMetersPerSecond)).in(Volts);
    double leftOutput =
        m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    double rightOutput =
        m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

    m_leftLeader.setVoltage(leftOutput + leftFeedforward);
    m_rightLeader.setVoltage(rightOutput + rightFeedforward);
  }
}
