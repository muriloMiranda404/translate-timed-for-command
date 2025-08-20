package frc.robot.subsystems.utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public interface DriverControllerIO {
    
    boolean activateMarcha();

    boolean isInvertedAlliance();

    Command driverRobot();

    double getLeftX();

    double getLeftY();

    double getRightX();

    double getRightY();

    double ConfigureInputs( int choose);
}
