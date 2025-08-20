package frc.robot.subsystems.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Controllers;
import frc.robot.subsystems.SwerveSubsystem;

public class DriverController implements DriverControllerIO{

    CommandXboxController controller;
    SwerveSubsystem swerve;

    public DriverController(){
        controller = new CommandXboxController(0);
        swerve = SwerveSubsystem.getInstance();
    }

    public double ConfigureInputs(int choose){

        // double marcha;
        // double invert = DriverStation.getAlliance().get() == Alliance.Red ? -1.0 : 1.0;
        // double gatilho = 0.7 + (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());

        //     marcha = gatilho;

        //     if(marcha <= 0){
        //         marcha *= -1.0;
        //     }

            switch (choose) {
                case 1:
                    
                    return controller.getLeftY();
            
                case 2:
                
                    return controller.getLeftX();

                case 3:
                
                    return controller.getRightX();
        }
        return choose;
    }

    @Override
    public boolean activateMarcha() {
     return ((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) != 0);
    }

    @Override
    public boolean isInvertedAlliance() {
        return DriverStation.getAlliance().get() == Alliance.Red;
    }

    @Override
    public Command driverRobot() {

        double marcha;
        double invert;

        // if(activateMarcha() == true){
        //     marcha = (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) + 0.7;

        //     if(marcha < 0.0){
        //         marcha = 0.3;
        //     }
        // } else {
        //     marcha = 0.7;
        // }

        // if(isInvertedAlliance() == true){
        //     invert = -1.0;
        // } else {
        //     invert = 1.0;
        // }

        double leftX = controller.getLeftY();
        double leftY = controller.getLeftX();
        double rightX = controller.getRightX();

        Command drive = swerve.driveCommand(
            () -> MathUtil.applyDeadband(leftY, Controllers.DEADBAND),
            () -> MathUtil.applyDeadband(leftX, Controllers.DEADBAND),
            () -> MathUtil.applyDeadband(rightX, Controllers.DEADBAND));

            return drive;
    }

    @Override
    public double getLeftX() {
        return controller.getLeftX();
    }

    @Override
    public double getLeftY() {
        return controller.getLeftY();
    }

    @Override
    public double getRightX() {
        return controller.getRightX();
    }

    @Override
    public double getRightY() {
       return controller.getRightY();
    }
}
