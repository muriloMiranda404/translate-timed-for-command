package frc.robot.subsystems.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriverController extends CommandXboxController{
    
    public DriverController(int id){
        super(id);
    }

    public double ConfigureInputs(boolean active, int choose){

        double marcha;
        double invert = DriverStation.getAlliance().get() == Alliance.Red ? -1.0 : 1.0;
        double gatilho = 0.7 + (super.getRightTriggerAxis() - super.getLeftTriggerAxis());

        if(active == true){ 

            marcha = gatilho;

            // if(marcha <= 0){
            //     marcha *= -1.0;
            // }

            switch (choose) {
                case 1:
                    
                    return super.getLeftY() * invert * marcha;
            
                case 2:
                
                    return super.getLeftX() * invert  * marcha;

                case 3:
                
                    return super.getRightX() * marcha;
            }
        }
        return choose;
    }
}
