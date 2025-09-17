// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Setpoints_CMD;
import frc.robot.Constants.ElevatorConstants;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SetpointConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Handoff_Elevator_CMD extends Command {
  /** Creates a new Elevator_L1. */
  ElevatorSubsystem m_elevatorSubsystem;
  double m_elevatorSetpoint;
  public Handoff_Elevator_CMD(ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.setElevatorMotionMagic(3.5, 5, ElevatorConstants.kElevatorMotionMagicJerk);
    m_elevatorSubsystem.setElevatorPosition(m_elevatorSetpoint);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.elevatorStop();
    m_elevatorSubsystem.setElevatorMotionMagic(ElevatorConstants.kElevatorMotionMagicAcceleration,ElevatorConstants.kElevatorMotionMagicCruiseVelocity,ElevatorConstants.kElevatorMotionMagicJerk);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
