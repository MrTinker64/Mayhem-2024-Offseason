// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

public class DriveIOSim implements DriveIO {
  private DifferentialDrivetrainSim sim =
      DifferentialDrivetrainSim.createKitbotSim(
          KitbotMotor.kDualCIMPerSide, KitbotGearing.k5p95, KitbotWheelSize.kSixInch, null);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    sim.update(0.02);
    inputs.leftPosition = sim.getLeftPositionMeters() / DriveConstants.WHEEL_RADIUS;
    inputs.rightPosition = sim.getRightPositionMeters() / DriveConstants.WHEEL_RADIUS;
    inputs.leftVelocity = sim.getLeftVelocityMetersPerSecond() / DriveConstants.WHEEL_RADIUS;
    inputs.rightVelocity = sim.getLeftVelocityMetersPerSecond() / DriveConstants.WHEEL_RADIUS;

    inputs.leftCurrentAmps = sim.getLeftCurrentDrawAmps();
    inputs.rightCurrentAmps = sim.getRightCurrentDrawAmps();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.rightAppliedVolts = rightAppliedVolts;
  }

  @Override
  public void arcadeDrive(double xSpeed, double omegaRotation) {
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, omegaRotation, true);
    leftAppliedVolts = MathUtil.clamp(speeds.left * 12.0, -12.0, 12.0);
    rightAppliedVolts = MathUtil.clamp(speeds.right * 12.0, -12.0, 12.0);
    sim.setInputs(leftAppliedVolts, rightAppliedVolts);
  }
}
