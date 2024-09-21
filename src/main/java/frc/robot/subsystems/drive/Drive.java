package frc.robot.subsystems.drive;

import com.ctre.phoenix.Logger;

import java.util.function.Supplier;

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
        io.arcadeDrive(1.0, 0);
    }

    public void driveCustom(double speed, double zRotation) {
        io.arcadeDrive(speed, zRotation);
    }

    public Command joystickDrive(Supplier<Double> xInput, Supplier<Double> omegaRotationInput) {
        return run(() -> {
            double xSpeed = xInput.get();

            if (xSpeed == 0) {
                fullStop();
            }

            double omegaRotation = omegaRotationInput.get();
            driveCustom(xSpeed, omegaRotation);
        }).withName("joystick drive");
    }
}
