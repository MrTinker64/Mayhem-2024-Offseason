package frc.robot.subsystems.drive;

import com.ctre.phoenix.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase{
    private final DriveIOReal io;

    public Drive(DriveIOReal io) {
        this.io = io;
    }

    public void periodic() {
        //TODO: Logs
        // io.updateInputs(inputs);
        // Logger.processInputs("Shooter/Feeder", inputs);
        // GeneralUtil.logSubsystem(this, "Shooter/Feeder");
    }

    public void fullStop() {
        io.stopDriveTrain();
    }

    public void driveForwardFullSpeed() {
        io.tankDrive(1.0, 1.0);
    }

    public void driveForwardCustomSpeed(double speed) {
        io.tankDrive(speed, speed);
    }

    public void customDriveSpeeds(double leftSpeed, double rightSpeed) {
        io.tankDrive(leftSpeed, rightSpeed);
    }

    
}
