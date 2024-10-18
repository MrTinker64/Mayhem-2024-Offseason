package frc.robot.subsystems.drive.Rollers;
import org.littletonrobotics.junction.AutoLog;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;


public interface RollerIO{
    @AutoLog


    public static class Rolle_IO_Inputs {
    
        public double r_Rot_Speed;        // rotations per second
        public double l_Rot_Speed;
         public double r_Applied_Volts ;  // in volts
         public double l_Applied_Volts;
    }

     public default void handle_Input(Rolle_IO_Inputs roller_Inputs){ }

     public default void run_L_Motor(double speed_In_Volts){}
     public default void run_R_Motor(double speed_In_Volts){}

     public default void stop_Motor(){}
} 
