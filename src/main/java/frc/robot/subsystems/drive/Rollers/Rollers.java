package frc.robot.subsystems.drive.Rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Rollers.RollerIO;
import frc.robot.subsystems.drive.DriveIOInputsAutoLogged;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.util.PoseManager;
//import frc.robot.subsystems.drive.Rollers.Roller_Constants_Dvid;

public class Rollers extends SubsystemBase{

 private final RollerIO io;
 public double  speed;
// private final rollerIOAutoLogged rollerInputs;



    public Rollers(RollerIO io, GyroIO gyroIO, PoseManager poseManager) {
        this.io = io;
    // this.gyroIO = gyroIO;
    // this.poseManager = poseManager;
    }


    public void periodic(){

    }

    private void run_volt_Left_Motor(double volts){
       io.run_L_Motor(volts);
    }

    private void run_volt_Right_Motor(double volts){
       io.run_R_Motor(volts);
    }
    private void run_Both_Motors(double volts){
         io.run_R_Motor(volts);
          io.run_L_Motor(volts);
    }

    
    private void stop_Both_Motors(){
        io.stop_Motor();
    }

    public Command run_Flywheel_Full_Speed (){
        return runOnce (() -> {
                run_Both_Motors(speed);
        });
    }
 public Command run_Flywheel_Half_Speed (){
        return runOnce (() -> {
                run_Both_Motors(speed/2);
        });
    }
 public Command run_Flywheel_x_Speed (double r_Speed, double l_Speed){
        return runOnce (() -> {
                run_volt_Left_Motor(l_Speed);
                run_volt_Right_Motor(r_Speed);
        });
    }
    /* Command notatiton
    public Command (Name) (){ return  [run command](() -> {
        [method]
    })
    
    }
  */

}

