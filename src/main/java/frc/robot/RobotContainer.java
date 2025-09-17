// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Set;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SetpointConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.AutoAlign.AlignToReef;
import frc.robot.commands.AutoAlign.AlignToReef.ReefSide;
import frc.robot.commands.Manual.Climber.ClimberManualDown;
import frc.robot.commands.Manual.Climber.ClimberManualUp;
import frc.robot.commands.Manual.Elevator.ElevatorManualDown;
import frc.robot.commands.Manual.Elevator.ElevatorManualUp;
import frc.robot.commands.Manual.EndEffector.Intake.EndEffectorManualIntake;
import frc.robot.commands.Manual.EndEffector.Intake.EndEffectorManualOuttake;
import frc.robot.commands.Manual.EndEffector.Intake.EndEffectorManualOuttake_L1;
import frc.robot.commands.Manual.EndEffector.Pivot.EndEffectorManualPivotDown;
import frc.robot.commands.Manual.EndEffector.Pivot.EndEffectorManualPivotUp;
import frc.robot.commands.Sequential.CLIMB_CMD;
import frc.robot.commands.Sequential.STOW_CMD;
import frc.robot.commands.Sequential.Intaking_CMDs.HP_EE_Intake_Sequence;
import frc.robot.commands.Sequential.Intaking_CMDs.HP_EE_Intake_Sequence_Reverse;
import frc.robot.commands.Sequential.Reef.L1_CMD;
import frc.robot.commands.Sequential.Reef.L2_CMD;
import frc.robot.commands.Sequential.Reef.L3_CMD;
import frc.robot.commands.Sequential.Reef.L4_CMD;
import frc.robot.commands.Sequential.Reef.REEF_CMD;
import frc.robot.commands.Setpoints_CMD.Elevator_Setpoint_CMD;
import frc.robot.commands.Setpoints_CMD.EndEffector_Setpoint_CMD;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final SendableChooser<Command> autoChooser;
  private SendableChooser<Double> speedChooser;



  private enum ReefState{
    Reef_A,
    Reef_B,
    Reef_C,
    Reef_D,
    Reef_E,
    Reef_F,
    Reef_G,
    Reef_H,
    Reef_I,
    Reef_J,
    Reef_K,
    Reef_L
  }





  private ReefState reefStateValue = ReefState.Reef_A;


  private enum EjectMethod{
    Normal_Eject,
    L1_Eject
  }

  private EjectMethod ejectMethodValue = EjectMethod.Normal_Eject;

  
 

  // Controller definitions
  CommandJoystick m_driverController = new CommandJoystick(0);
  CommandJoystick m_operatorController1 = new CommandJoystick(1);
  CommandJoystick m_operatorController2 = new CommandJoystick(2);

  // CommandXboxController m_operatorButtonBox = new CommandXboxController(2);


  // private final Trigger DRIVER_A_BUTTON = new Trigger(() -> m_driverController.getHID().getAButton());
  // private final Trigger DRIVER_B_BUTTON = new Trigger(() -> m_driverController.getHID().getBButton());
  // private final Trigger DRIVER_X_BUTTON = new Trigger(() -> m_driverController.getHID().getXButton());
  // private final Trigger DRIVER_Y_BUTTON = new Trigger(() -> m_driverController.getHID().getYButton());

  // private final Trigger DRIVER_POV_UP = new Trigger(m_driverController.povUp());
  // private final Trigger DRIVER_POV_DOWN = new Trigger(m_driverController.povDown());
  // private final Trigger DRIVER_POV_LEFT = new Trigger(m_driverController.povLeft());
  // private final Trigger DRIVER_POV_RIGHT = new Trigger(m_driverController.povRight());

  // private final Trigger DRIVER_LEFT_TRIGGER = new Trigger(m_driverController.leftTrigger());
  // private final Trigger DRIVER_RIGHT_TRIGGER = new Trigger(m_driverController.rightTrigger());
  // private final Trigger DRIVER_LEFT_BUMPER = new Trigger(m_driverController.leftBumper());
  // private final Trigger DRIVER_RIGHT_BUMPER = new Trigger(m_driverController.rightBumper());


  private final double DRIVER_LEFT_X_AXIS = m_driverController.getRawAxis(0);
  private final double DRIVER_LEFT_Y_AXIS = m_driverController.getRawAxis(1);

  private final double DRIVER_RIGHT_X_AXIS = m_driverController.getRawAxis(4);
  private final double DRIVER_RIGHT_Y_AXIS = m_driverController.getRawAxis(5);

  private final Trigger DRIVER_A_BUTTON = new Trigger(() -> m_driverController.getHID().getRawButton(1));
  private final Trigger DRIVER_B_BUTTON = new Trigger(() -> m_driverController.getHID().getRawButton(2));
  private final Trigger DRIVER_X_BUTTON = new Trigger(() -> m_driverController.getHID().getRawButton(3));
  private final Trigger DRIVER_Y_BUTTON = new Trigger(() -> m_driverController.getHID().getRawButton(4));

  private final Trigger DRIVER_POV_UP = new Trigger(m_driverController.povUp());
  private final Trigger DRIVER_POV_DOWN = new Trigger(m_driverController.povDown());
  private final Trigger DRIVER_POV_LEFT = new Trigger(m_driverController.povLeft());
  private final Trigger DRIVER_POV_RIGHT = new Trigger(m_driverController.povRight());

  private final Trigger DRIVER_LEFT_TRIGGER = new Trigger(m_driverController.axisGreaterThan(2, 0.25));
  private final Trigger DRIVER_RIGHT_TRIGGER = new Trigger(m_driverController.axisGreaterThan(3, 0.25));

  private final Trigger DRIVER_LEFT_BUMPER = new Trigger(() -> m_driverController.getHID().getRawButton(5));
  private final Trigger DRIVER_RIGHT_BUMPER = new Trigger(() -> m_driverController.getHID().getRawButton(6));

  private final Trigger DRIVER_START_BUTTON = new Trigger(() -> m_driverController.getHID().getRawButton(8));
  private final Trigger DRIVER_BACK_BUTTON = new Trigger(() -> m_driverController.getHID().getRawButton(7));

  private final Trigger DRIVER_M1_BUMPER = new Trigger(() -> m_driverController.getHID().getRawButton(0)); //TODO:
  private final Trigger DRIVER_M2_BUMPER = new Trigger(() -> m_driverController.getHID().getRawButton(0)); //TODO:

  private final Trigger DRIVER_M3_LEFT_UP_PADDLE = new Trigger(() -> m_driverController.getHID().getRawButton(0)); //TODO:
  private final Trigger DRIVER_M5_LEFT_DOWN_PADDLE = new Trigger(() -> m_driverController.getHID().getRawButton(0)); //TODO:

  private final Trigger DRIVER_M4_RIGHT_UP_PADDLE = new Trigger(() -> m_driverController.getHID().getRawButton(0)); //TODO:
  private final Trigger DRIVER_M6_RIGHT_DOWN_PADDLE = new Trigger(() -> m_driverController.getHID().getRawButton(0)); //TODO:






  // Subsystem definitions
  SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/falcon"));
  EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();
  ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();



  //Elevator Manual Commands
  ElevatorManualDown m_elevatorManualDown = new ElevatorManualDown(m_elevatorSubsystem);
  ElevatorManualUp m_elevatorManualUp = new ElevatorManualUp(m_elevatorSubsystem);

  //End Effector Manual Commands
  EndEffectorManualIntake m_endEffectorManualIntake = new EndEffectorManualIntake(m_endEffectorSubsystem);
  EndEffectorManualOuttake m_endEffectorManualOuttake = new EndEffectorManualOuttake(m_endEffectorSubsystem);
  EndEffectorManualOuttake_L1 m_EndEffectorManualOuttake_L1 = new EndEffectorManualOuttake_L1(m_endEffectorSubsystem);

  EndEffectorManualPivotDown m_endEffectorManualPivotDown = new EndEffectorManualPivotDown(m_endEffectorSubsystem);
  EndEffectorManualPivotUp m_endEffectorManualPivotUp = new EndEffectorManualPivotUp(m_endEffectorSubsystem);


  // Elevator Setpoint Commands
  Elevator_Setpoint_CMD m_elevatorL1 = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kL1ElevatorSetpoint);
  Elevator_Setpoint_CMD m_elevatorL2 = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kL2ElevatorSetpoint);
  Elevator_Setpoint_CMD m_elevatorL3 = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kL3ElevatorSetpoint);
  Elevator_Setpoint_CMD m_elevatorL4 = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kL4ElevatorSetpoint);
  Elevator_Setpoint_CMD m_elevatorStow = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kStowElevatorSetpoint);
  Elevator_Setpoint_CMD m_elevatorHP = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kHPElevatorSetpoint);

  // End Effector Setpoint Commands
  EndEffector_Setpoint_CMD m_endEffectorL1 = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kL1EndEffectorSetpoint);
  EndEffector_Setpoint_CMD m_endEffectorL2 = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kL2EndEffectorSetpoint);
  EndEffector_Setpoint_CMD m_endEffectorL3 = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kL3EndEffectorSetpoint);
  EndEffector_Setpoint_CMD m_endEffectorL4 = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kL4EndEffectorSetpoint);
  EndEffector_Setpoint_CMD m_endEffectorStow = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kStowEndEffectorSetpoint);
  EndEffector_Setpoint_CMD m_endEffectorHP = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kHPEndEffectorSetpoint);
  // Sequence Commands

  HP_EE_Intake_Sequence m_HP_EE_Intake_Sequence = new HP_EE_Intake_Sequence(m_elevatorSubsystem, m_endEffectorSubsystem);

  HP_EE_Intake_Sequence_Reverse m_HP_EE_Intake_Sequence_Reverse = new HP_EE_Intake_Sequence_Reverse(m_elevatorSubsystem, m_endEffectorSubsystem);

  STOW_CMD m_STOW_CMD = new STOW_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);
  L1_CMD m_L1_CMD = new L1_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);
  L2_CMD m_L2_CMD = new L2_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);
  L3_CMD m_L3_CMD = new L3_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);
  L4_CMD m_L4_CMD = new L4_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);

  // Algae Removal Commands
  public REEF_CMD m_L3_Algae_Removal = new REEF_CMD(m_elevatorSubsystem, m_endEffectorSubsystem, SetpointConstants.kEndEffectorL3AlgaeRemovalSetpoint, SetpointConstants.kElevatorL3AlgaeRemovalSetpoint);
  public REEF_CMD m_L2_Algae_Removal = new REEF_CMD(m_elevatorSubsystem, m_endEffectorSubsystem,SetpointConstants.kEndEffectorL2AlgaeRemovalSetpoint, SetpointConstants.kElevatorL2AlgaeRemovalSetpoint);


  LEDSubsystem m_ledSubsystem = new LEDSubsystem( m_endEffectorSubsystem, m_elevatorSubsystem, m_swerveSubsystem,
                                                this);
  
  ClimberManualDown m_climberManualDown = new ClimberManualDown(m_climberSubsystem);
  ClimberManualUp m_climberManualUp = new ClimberManualUp(m_climberSubsystem);

  CLIMB_CMD m_climb_CMD = new CLIMB_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);

  AlignToReef m_alignToReef = new AlignToReef(m_swerveSubsystem);

  

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                                                                () -> DRIVER_LEFT_Y_AXIS * -1,
                                                                () -> DRIVER_LEFT_X_AXIS * -1)
                                                            .withControllerRotationAxis(() -> DRIVER_RIGHT_X_AXIS * -1)
                                                            .deadband(OperatorConstants.kDeadband)
                                                            .scaleTranslation(SpeedConstants.kNormalRobotTranslationSpeed)
                                                            .scaleRotation(SpeedConstants.kNormalRobotRotationSpeed)
                                                            .allianceRelativeControl(true);
  // Trigger m_speedTrigger = new Trigger(() -> {
  //   Double selectedSpeed = speedChooser.getSelected();
  //   if (selectedSpeed != 0.7) {
  //     driveAngularVelocity.scaleTranslation(selectedSpeed);
  //   }
  //   return false; // Adjust logic as needed for the trigger condition
  // });
  SwerveInputStream driveAngularVelocity_SLOW = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                                                                ()-> DRIVER_LEFT_Y_AXIS * -1,
                                                                () -> DRIVER_LEFT_X_AXIS * -1)
                                                            .withControllerRotationAxis(() -> DRIVER_RIGHT_X_AXIS * -1)
                                                            .deadband(OperatorConstants.kDeadband)
                                                            .scaleTranslation(0.25)
                                                            .scaleRotation(0.25)
                                                            .allianceRelativeControl(true);

    SwerveInputStream driveAngularVelocity_Elevator_Creep = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                                                                () -> DRIVER_LEFT_Y_AXIS * -1,
                                                                () -> DRIVER_RIGHT_Y_AXIS * -1)
                                                            .withControllerRotationAxis(() -> DRIVER_RIGHT_X_AXIS * -1)
                                                            .deadband(OperatorConstants.kDeadband)
                                                            .scaleTranslation(0.1)
                                                            .scaleRotation(0.1)
                                                            .allianceRelativeControl(true);
                                                            

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  // SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
  //                                                                                            m_driverController::getRightY)
  //                                                          .headingWhile(true);

  SwerveInputStream driveRobotOriented = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                                                            getYAxisPOV(),
                                                            getXAxisPOV())
                                                            .withControllerRotationAxis(getRotAxis())
                                                            .scaleTranslation(SpeedConstants.kRobotNudgeSpeed)
                                                            .allianceRelativeControl(false)
                                                            .robotRelative(true); 

  L4_CMD m_L4_CMD_AUTO = new L4_CMD(m_elevatorSubsystem, m_endEffectorSubsystem); 

  // Trigger m_elevatorCreep_CMD = new Trigger(()-> m_elevatorSubsystem.getElevatorPositionRotations() > SetpointConstants.kL2ElevatorSetpoint);





  private void speedChooserSetup() {
    speedChooser = new SendableChooser<Double>();
    speedChooser.setDefaultOption("70%", 0.7);
    speedChooser.addOption("95%", 0.95);
    speedChooser.addOption("90%", 0.9);
    speedChooser.addOption("85%", 0.85);
    speedChooser.addOption("80%", 0.8);
    speedChooser.addOption("75%", 0.75);
    speedChooser.addOption("60%", 0.6);
    speedChooser.addOption("50%", 0.5);
    speedChooser.addOption("MROC 40%", 0.4);
    speedChooser.addOption("30%", 0.3);
    speedChooser.addOption("25%", 0.25);
    SmartDashboard.putData("Speed Chooser", speedChooser);

    speedChooser.onChange(driveAngularVelocity::scaleTranslation);
  }
 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() 
  { 
    speedChooserSetup();
    
    NamedCommands.registerCommand("L4_CMD", new L4_CMD(m_elevatorSubsystem, m_endEffectorSubsystem));
    NamedCommands.registerCommand("L3_CMD", new L3_CMD(m_elevatorSubsystem, m_endEffectorSubsystem));

    NamedCommands.registerCommand("EndEffector_Eject_Coral", new EndEffectorManualOuttake(m_endEffectorSubsystem).withTimeout(0.4));
    NamedCommands.registerCommand("EndEffector_Eject_Coral_L3", new EndEffectorManualOuttake(m_endEffectorSubsystem).withTimeout(0.65));
    NamedCommands.registerCommand("EndEffector_Stow_1_Timeout", new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kStowEndEffectorSetpoint).withTimeout(1));
    NamedCommands.registerCommand("EndEffector_Stow_0.5_Timeout", new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kStowEndEffectorSetpoint).withTimeout(0.6));
    NamedCommands.registerCommand("EndEffector_Stow_1.5_Timeout", new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kStowEndEffectorSetpoint).withTimeout(1.5));
    
    NamedCommands.registerCommand("STOW_CMD", new ParallelCommandGroup(
                                                      new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kStowEndEffectorSetpoint),
                                                      new SequentialCommandGroup(
                                                        new WaitCommand(0.5),
                                                        new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kStowElevatorSetpoint)
                                                      )
                                                      ).withTimeout(1.5));
    NamedCommands.registerCommand("HP_EE_INTAKE", new HP_EE_Intake_Sequence(m_elevatorSubsystem, m_endEffectorSubsystem).until(()->m_endEffectorSubsystem.hasCoral()));

    NamedCommands.registerCommand("Stow_&_EE_Intake", new STOW_CMD(m_elevatorSubsystem, m_endEffectorSubsystem)
    .withTimeout(1)
    .andThen(
                                                        new HP_EE_Intake_Sequence(m_elevatorSubsystem, m_endEffectorSubsystem).until(()->m_endEffectorSubsystem.hasCoral())
                                                      ));

    // driveAngularVelocity.scaleTranslation(speedChooser.getSelected());

    NamedCommands.registerCommand("DA2", new ParallelCommandGroup(
      new REEF_CMD(m_elevatorSubsystem, m_endEffectorSubsystem, SetpointConstants.kEndEffectorL2AlgaeRemovalSetpoint, SetpointConstants.kElevatorL2AlgaeRemovalSetpoint),
      new EndEffectorManualIntake(m_endEffectorSubsystem)
    ));



    configureBindings();



    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser();

    
    SmartDashboard.putData("Auto Chooser", autoChooser);

    m_swerveSubsystem.replaceSwerveModuleFeedforward(OperatorConstants.kSSwerveFeedforward, OperatorConstants.kVSwerveFeedforward, OperatorConstants.kASwerveFeedforward);
  }



  private DoubleSupplier getXAxisPOV(){
    return () -> {
      if(DRIVER_POV_LEFT.getAsBoolean()) return 1;
      if(DRIVER_POV_RIGHT.getAsBoolean()) return -1;
      return 0;
    };
  }

  private DoubleSupplier getYAxisPOV(){
    return () -> {
      if(DRIVER_POV_DOWN.getAsBoolean()) return 0;
      if(DRIVER_POV_UP.getAsBoolean()) return 0;
      return 0;
    };

  }

  private DoubleSupplier getRotAxis(){
    return () -> {
      if(DRIVER_POV_DOWN.getAsBoolean()) return 0;
      if(DRIVER_POV_UP.getAsBoolean()) return 0;
      return 0;
    };
  }



  @AutoLogOutput(key = "General/ReefState")
  private String getReefStateString(){
    return reefStateValue.toString();
  }

 
  private Command getDesiredOuttakeCMD(){
    return
    switch (ejectMethodValue) {
      case Normal_Eject -> m_endEffectorManualOuttake;
      case L1_Eject -> m_EndEffectorManualOuttake_L1;
    };
  }

  public void scaleTranslation() {
    driveAngularVelocity.scaleTranslation(speedChooser.getSelected());
  }





   



  

  // Method to configure bindings
  private void configureBindings()
  {

    // Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity); 
    Command driveRobotOrientedNudge = m_swerveSubsystem.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedAnglularVelocity_SLOW = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity_SLOW);
    



    if (RobotBase.isSimulation())
    {
      m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else
    {
      m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      // Simulation driver bindings
      
    }
    if (DriverStation.isTest())
    {
      // Test driver bindings
      m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

    } else
    {


     
      
    // Driver Controls

      // DRIVER_A_BUTTON.onTrue(
      //   Commands.runOnce(m_swerveSubsystem :: zeroGyroWithAlliance)
      // );




      DRIVER_A_BUTTON.whileTrue(
        m_alignToReef.AlignToTheClosestBranch(ReefSide.LEFT)
      );

      DRIVER_B_BUTTON.whileTrue(
        m_alignToReef.AlignToTheClosestBranch(ReefSide.RIGHT)
      );
      

      DRIVER_POV_RIGHT.whileTrue(
        Commands.runOnce(() -> {
          driveRobotOrientedNudge.schedule();
      })
      ).whileFalse(
        Commands.runOnce(() -> {
          driveRobotOrientedNudge.cancel();
        })
      );

      DRIVER_POV_LEFT.whileTrue(
        Commands.runOnce(() -> {
          driveRobotOrientedNudge.schedule();
      })
      ).whileFalse(
        Commands.runOnce(() -> {
          driveRobotOrientedNudge.cancel();
        })
      );
// Lucas Holl is lead programmer
      //DRIVER_POV_DOWN.whileTrue(Commands.runOnce(m_swerveSubsystem::lock, m_swerveSubsystem).repeatedly());

      m_operatorController1.button(6).whileTrue(Commands.run(()->m_driverController.setRumble(RumbleType.kBothRumble, 1)))
      .whileFalse(Commands.run(()->m_driverController.setRumble(RumbleType.kBothRumble, 0)));




    


      DRIVER_LEFT_TRIGGER.onTrue(m_HP_EE_Intake_Sequence);

     //DRIVER_POV_UP.onTrue(m_HP_EE_Intake_Sequence_Reverse);



      DRIVER_RIGHT_TRIGGER.whileTrue(m_endEffectorManualOuttake);

      DRIVER_RIGHT_BUMPER.whileTrue(Commands.runOnce(()->getDesiredOuttakeCMD().schedule())).whileFalse(Commands.runOnce(()->getDesiredOuttakeCMD().cancel()));



      

      // Cancel All Commands
      DRIVER_X_BUTTON.whileTrue(Commands.run(()->CommandScheduler.getInstance().cancelAll()));

      // Override Intake Command
      DRIVER_Y_BUTTON.whileTrue(m_endEffectorManualIntake);
      
      
      // Climber Commands
      DRIVER_START_BUTTON.whileTrue(m_climberManualUp);
      m_operatorController2.button(10).whileTrue(m_climberManualDown);
      DRIVER_BACK_BUTTON.whileTrue(m_climberManualDown);



      DRIVER_LEFT_BUMPER.whileTrue(driveFieldOrientedAnglularVelocity_SLOW);







    // Operator controls


        // Operator L1 
        m_operatorController1.button(OperatorConstants.kButtonBox_L1_Button_Port1).whileTrue(
          Commands.run(() -> {
            m_elevatorManualDown.cancel();
            m_L1_CMD.schedule();
            m_endEffectorStow.cancel();
            m_HP_EE_Intake_Sequence.cancel();
            new InstantCommand(()->ejectMethodValue = EjectMethod.L1_Eject).schedule();
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            m_L1_CMD.cancel();
            m_elevatorManualDown.schedule();
            m_endEffectorStow.schedule();
            m_EndEffectorManualOuttake_L1.cancel();
            new InstantCommand(()->ejectMethodValue = EjectMethod.Normal_Eject).schedule();
          })
        );

        // m_operatorController1.button(OperatorConstants.kButtonBox_L1_Button_Port1).onTrue(
        //   new InstantCommand(()->ejectMethodValue = EjectMethod.Normal_Eject)
        // ).onFalse(
        //   new InstantCommand(()->ejectMethodValue = EjectMethod.L1_Eject)
        // );




      // Operator L2
        m_operatorController1.button(OperatorConstants.kButtonBox_L2_Button_Port1).whileTrue(
          Commands.run(() -> {
            m_elevatorManualDown.cancel();
            m_L2_CMD.schedule();
            m_endEffectorStow.cancel();
            m_HP_EE_Intake_Sequence.cancel();
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            m_L2_CMD.cancel();
            m_elevatorManualDown.schedule();
            m_endEffectorStow.schedule();
          })
        );

        // Operator L3
        m_operatorController1.button(OperatorConstants.kButtonBox_L3_Button_Port1).whileTrue(
          Commands.run(() -> {
            m_elevatorManualDown.cancel();
            m_L3_CMD.schedule();
            m_endEffectorStow.cancel();
            m_HP_EE_Intake_Sequence.cancel();
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            m_L3_CMD.cancel();
            m_elevatorManualDown.schedule();
            m_endEffectorStow.schedule();
          })
        );


        // Operator L4
        m_operatorController2.button(OperatorConstants.kButtonBox_L4_Button_Port2).whileTrue(
          Commands.run(() -> {
            m_elevatorManualDown.cancel();
            m_L4_CMD.schedule();
            m_endEffectorStow.cancel();
            m_HP_EE_Intake_Sequence.cancel();
           
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            m_L4_CMD.cancel();
            m_elevatorManualDown.schedule();
            m_endEffectorStow.schedule();
          })
        );

        // Operator De-Algae at L2
        m_operatorController2.button(11).whileTrue(
          Commands.run(() -> {
            // m_elevatorManualDown.cancel();
            // m_endEffectorStow.cancel();
            // m_HP_EE_Intake_Sequence.cancel();
            CommandScheduler.getInstance().cancelAll();
            m_endEffectorManualIntake.schedule();
            m_L2_Algae_Removal.schedule();
            
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            m_endEffectorManualIntake.cancel();
            m_L2_Algae_Removal.cancel();
            m_elevatorManualDown.schedule();
            m_endEffectorStow.schedule();
          })
        );

        // Operator De-Algae at L3
        m_operatorController1.button(10).whileTrue(
          Commands.run(() -> {
            m_elevatorManualDown.cancel();
            m_endEffectorStow.cancel();
            m_HP_EE_Intake_Sequence.cancel();
            m_endEffectorManualIntake.schedule();
            m_L3_Algae_Removal.schedule();
            
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            m_endEffectorManualIntake.cancel();
            m_L3_Algae_Removal.cancel();
            m_elevatorManualDown.schedule();
            m_endEffectorStow.schedule();
          })
        );




        // Operator Elevator Manual Up
        m_operatorController2.button(OperatorConstants.kButtonBox_ELEVATOR_MANUAL_UP_Button_Port2).whileTrue(
          Commands.run(() -> {
            CommandScheduler.getInstance().cancelAll();
            m_elevatorManualUp.schedule();
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            m_elevatorManualUp.cancel();
          })
        );

        // Operator Elevator Manual Down
        m_operatorController1.button(OperatorConstants.kButtonBox_ELEVATOR_MANUAL_DOWN_Button_Port1).whileTrue(
          Commands.run(() -> {
            CommandScheduler.getInstance().cancelAll();
            m_elevatorManualDown.schedule();
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            m_elevatorManualDown.cancel();
          })
        );
        }


        // Operator Climbing Position
        m_operatorController2.button(8).whileTrue(
          Commands.run(() -> {
            CommandScheduler.getInstance().cancelAll();
            m_climb_CMD.schedule();
            
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
            m_elevatorManualDown.schedule();
            m_endEffectorStow.schedule();
          })
        );
   
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to runOnce in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be runOnce in autonomous
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    m_swerveSubsystem.setMotorBrake(brake);
  }
}
