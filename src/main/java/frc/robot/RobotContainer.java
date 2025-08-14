// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Controllers;
import frc.robot.Constants.Outros;
import frc.robot.Constants.Elevator.ElevatorPositions;
import frc.robot.Constants.Intake.IntakePositions;
import frc.robot.commands.ElevatorPositionCommand;
import frc.robot.commands.IntakePositionCommand;
import frc.robot.commands.IntakeSpeedCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private static final CommandXboxController DriveJoystick = new CommandXboxController(Controllers.DRIVE_CONTROLLER);
  private static final XboxController IntakeJoystick = new XboxController(Controllers.INTAKE_CONTROLLER);

  private static final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  
  private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private static final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

  SendableChooser<Command> AutoChooser;

  public RobotContainer() {

    swerve.setDefaultCommand(swerve.driveCommand(
      () -> MathUtil.applyDeadband(DriveJoystick.getLeftY(), Controllers.DEADBAND), 
      () -> MathUtil.applyDeadband(DriveJoystick.getLeftX(), Controllers.DEADBAND), 
      () -> MathUtil.applyDeadband(DriveJoystick.getRightX(), Controllers.DEADBAND)));


    configureDriveBindings();
    configureIntakeBindings();

    AutoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", AutoChooser);
  }

  private void configureDriveBindings() {
  }
  
  
  private void configureIntakeBindings(){ 

    //pegar e empurra o coral
    NamedCommands.registerCommand("GET CORAL", new IntakeSpeedCommand(intake, true));
    NamedCommands.registerCommand("THROW CORAL", new IntakeSpeedCommand(intake, 0.8));

    //intake
    NamedCommands.registerCommand("DEFAULT POSITION", new IntakePositionCommand(intake, IntakePositions.DEFAULT_POSITION));
    NamedCommands.registerCommand("ABERTURA SIMPLES", new IntakePositionCommand(intake, IntakePositions.ABERTURA_COMUMM));
    NamedCommands.registerCommand("PUT CORAL", new IntakePositionCommand(intake, IntakePositions.PUT_CORAL));
    NamedCommands.registerCommand("ABERTURA L4", new IntakePositionCommand(intake, IntakePositions.OPEN_L4));
    NamedCommands.registerCommand("CONTROLL BALL", new IntakePositionCommand(intake, IntakePositions.CONTROL_BALL));

    //elevador
    NamedCommands.registerCommand("HOME", new ElevatorPositionCommand(elevator, ElevatorPositions.HOME));
    NamedCommands.registerCommand("L2", new ElevatorPositionCommand(elevator, ElevatorPositions.L2));
    NamedCommands.registerCommand("L3", new ElevatorPositionCommand(elevator, ElevatorPositions.L3));
    NamedCommands.registerCommand("L4", new ElevatorPositionCommand(elevator, ElevatorPositions.L4));
    NamedCommands.registerCommand("ALGAE L2", new ElevatorPositionCommand(elevator, ElevatorPositions.ALGAE_L2));
    NamedCommands.registerCommand("ALGAE L3", new ElevatorPositionCommand(elevator, ElevatorPositions.ALGAE_L3));

    NamedCommands.registerCommand("L1 FULL", new SequentialCommandGroup(
      NamedCommands.getCommand("ABERTURA SIMPLES"),
      NamedCommands.getCommand("HOME"),
      NamedCommands.getCommand("DEFAULT POSITION")
    ));

    NamedCommands.registerCommand("L2 FULL", new SequentialCommandGroup(
      NamedCommands.getCommand("PUT CORAL"),
      NamedCommands.getCommand("L2")
    ));

    NamedCommands.registerCommand("L3 FULL", new SequentialCommandGroup(
      NamedCommands.getCommand("PUT CORAL"),
      NamedCommands.getCommand("L3")
    ));

    NamedCommands.registerCommand("L4 FULL", new SequentialCommandGroup(
      NamedCommands.getCommand("PUT CORAL"),
      NamedCommands.getCommand("L4"),
      NamedCommands.getCommand("ABERTURA L4")
    ));

    NamedCommands.registerCommand("ALGAE 1", new SequentialCommandGroup(
      NamedCommands.getCommand("CONTROLL BALL"),
      NamedCommands.getCommand("ALGAE L2")
    ));

    NamedCommands.registerCommand("ALGAE 2", new SequentialCommandGroup(
      NamedCommands.getCommand("CONTROLL BALL"),
      NamedCommands.getCommand("ALGAE L3")
    ));

    NamedCommands.registerCommand("PROCESSADOR", new SequentialCommandGroup(
      NamedCommands.getCommand("CONTROLL BALL"),
      NamedCommands.getCommand("HOME")
    ));

    new JoystickButton(IntakeJoystick, 1).onTrue(NamedCommands.getCommand("L1 FULL"));
    new JoystickButton(IntakeJoystick, 2).onTrue(NamedCommands.getCommand("L2 FULL"));
    new JoystickButton(IntakeJoystick, 3).onTrue(NamedCommands.getCommand("L3 FULL"));
    new JoystickButton(IntakeJoystick, 4).onTrue(NamedCommands.getCommand("L4 FULL"));

    new JoystickButton(IntakeJoystick, 5).onTrue(NamedCommands.getCommand("GET CORAL"));
    new JoystickButton(IntakeJoystick, 6).whileTrue(NamedCommands.getCommand("THROW CORAL"));

    new JoystickButton(IntakeJoystick, 7).onTrue(NamedCommands.getCommand("PROCESSADOR"));
    new JoystickButton(IntakeJoystick, 8).onTrue(NamedCommands.getCommand("ALGAE 1"));
    new JoystickButton(IntakeJoystick, 9).onTrue(NamedCommands.getCommand("ALGAE 2"));
}

  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand(Outros.AUTO, true);
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}
