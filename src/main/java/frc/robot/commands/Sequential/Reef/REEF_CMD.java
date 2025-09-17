// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequential.Reef;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SetpointConstants;
import frc.robot.commands.Setpoints_CMD.Elevator_Setpoint_CMD;
import frc.robot.commands.Setpoints_CMD.EndEffector_Setpoint_CMD;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class REEF_CMD extends SequentialCommandGroup {
  /** Creates a new REEF_CMD. */
  EndEffectorSubsystem m_endEffectorSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  // CoralGroundIntakeSubsystem m_coralGroundIntakeSubsystem;
  double m_endEffectorSetpoint;
  double m_elevatorSetpoint;
  public REEF_CMD(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, double endEffectorSetpoint, double elevatorSetpoint) {
    // Add REEF_CMD commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_elevatorSubsystem = elevatorSubsystem;
    m_endEffectorSubsystem = endEffectorSubsystem;
    m_endEffectorSetpoint = endEffectorSetpoint;
    m_elevatorSetpoint = elevatorSetpoint;

    addCommands(
    new ParallelCommandGroup(
                 new Elevator_Setpoint_CMD(m_elevatorSubsystem, m_elevatorSetpoint), 
                 new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, m_endEffectorSetpoint)));
  }
}
