// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import javax.swing.plaf.RootPaneUI;

import org.littletonrobotics.junction.AutoLogOutput;
import org.opencv.features2d.FlannBasedMatcher;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.PWM1Configs;
import com.ctre.phoenix6.configs.PWM2Configs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.SetpointConstants;
import frc.robot.Elastic;
import frc.robot.commands.Setpoints_CMD.EndEffector_Setpoint_CMD;

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new EndEffector. */

  // End Effector Intake
  public TalonFXS endEffectorIntake;
  // End Effector Pivot
  private TalonFX endEffectorPivot;



  // End Effector Configs
  private TalonFXSConfiguration intakeConfigs;
  // Pivot Configs
  private TalonFXConfiguration pivotConfigs;

  // Intake Beam Break
  // private TimeOfFlight intakeSensor;

  // Voltage Request
  private VoltageOut m_voltageRequest;

  //MotionMagic Voltage Request
  private MotionMagicVoltage m_motionRequest;

  //Throughbore
  private CANcoder endEffectorEncoder;
  private CANcoderConfiguration endEffectorEncoderConfigs;


  public EndEffectorSubsystem() {
    
    // End Effector Intake
    endEffectorIntake = new TalonFXS(EndEffectorConstants.kEndEffectorIntakeID);
    // End Effector Pivot
    endEffectorPivot = new TalonFX(EndEffectorConstants.kEndEffectorPivotID);


    
    
    endEffectorEncoder = new CANcoder(EndEffectorConstants.kEndEffectorEncoderID);
    endEffectorEncoderConfigs = new CANcoderConfiguration()
                          .withMagnetSensor(new MagnetSensorConfigs()
                                            .withMagnetOffset(EndEffectorConstants.kEncoderOffset)
                                            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                                            .withAbsoluteSensorDiscontinuityPoint(1)
                                            ); 
    endEffectorEncoder.getConfigurator().apply(endEffectorEncoderConfigs);




    // Intake Configs
    intakeConfigs = new TalonFXSConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                              .withInverted(InvertedValue.CounterClockwise_Positive)
                                              .withNeutralMode(NeutralModeValue.Brake))
                        .withCommutation(new CommutationConfigs()
                                              .withMotorArrangement(MotorArrangementValue.Minion_JST))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                              .withSupplyCurrentLimit(15));

    // Apply Intake Configs
    endEffectorIntake.getConfigurator().apply(intakeConfigs);

    // Pivot Configs
    pivotConfigs = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.CounterClockwise_Positive)
                                            .withNeutralMode(NeutralModeValue.Brake))
                        .withSlot0(new Slot0Configs()
                                        .withKP(EndEffectorConstants.kEndEffectorPivotPIDValueP)
                                        .withKI(EndEffectorConstants.kEndEffectorPivotPIDValueI)
                                        .withKD(EndEffectorConstants.kEndEffectorPivotPIDValueD))
                        .withMotionMagic(new MotionMagicConfigs()
                                            .withMotionMagicCruiseVelocity(EndEffectorConstants.kEndEffectorPivotMotionMagicCruiseVelocity)
                                            .withMotionMagicAcceleration(EndEffectorConstants.kEndEffectorPivotMotionMagicAcceleration)
                                            .withMotionMagicJerk(EndEffectorConstants.kEndEffectorPivotMotionMagicJerk))
                        .withFeedback(new FeedbackConfigs()
                                            .withFeedbackRemoteSensorID(EndEffectorConstants.kEndEffectorEncoderID)
                                            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                                            .withSensorToMechanismRatio(EndEffectorConstants.kSensorToMechanismRatio)
                                            .withRotorToSensorRatio(EndEffectorConstants.kRotorToSensorRatio))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                            .withStatorCurrentLimit(Units.Amps.of(EndEffectorConstants.kEndEffectorPivotCurrentLimit)));

                        
                        
    // Apply Pivot Configs
    endEffectorPivot.getConfigurator().apply(pivotConfigs);
  
    // SysID voltage request
    m_voltageRequest = new VoltageOut(0);
    // Motion Magic motion request
    m_motionRequest = new MotionMagicVoltage(0).withSlot(0).withFeedForward(EndEffectorConstants.kEndEffectorFeedForward);




  }


  // End Effector Intake
  public void rollersIntake() {
    endEffectorIntake.set(EndEffectorConstants.kEndEffectorSpeed);
  }

  public void rollersOuttake_L1() {
    endEffectorIntake.set(-0.15);
  }

  // End Effector Outtake
  public void rollersOuttake() {
    endEffectorIntake.set(-EndEffectorConstants.kEndEffectorSpeed);
  }

  // End Effector Stop
  public void rollersStop() {
    endEffectorIntake.set(0);
  }


  // End Effector Pivot Up
  public void EndEffectorPivotUp() {
    endEffectorPivot.set(EndEffectorConstants.kPivotSpeed);
  }

  // End Effector Pivot Down
  public void EndEffectorPivotDown() {
    endEffectorPivot.set(-EndEffectorConstants.kPivotSpeed);
  }

  // End Effector Pivot Stop
  public void EndEffectorPivotStop() {
    endEffectorPivot.set(0);
  } 



  public Command intakeWithCurrent(){
    return Commands.run(()->{ 
      if(this.endEffectorIntake.getStatorCurrent().getValueAsDouble() > EndEffectorConstants.kEndEffectorCurrentSpike){
        rollersStop();
      }else
        rollersIntake();
    }
    ).until(() -> this.endEffectorIntake.getStatorCurrent().getValueAsDouble() > EndEffectorConstants.kEndEffectorCurrentSpike);
  }

  // Set Pivot Postion
  public void setPivotPosition(double position){
    endEffectorPivot.setControl(m_motionRequest.withPosition(position));
  }


  // SysID definition
  private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(
          null,            // Use default ramp rate (1 V/s)
          Volts.of(4),    // Reduce dynamic step voltage to 4 to prevent brownout
          Seconds.of(10),  // Use default timeout (10 s)
          // Log state with Phoenix SignalLogger class
          (state) -> SignalLogger.writeString("End Effector State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
          (volts) -> endEffectorPivot.setControl(m_voltageRequest.withOutput(volts.in(Volts))),
          null,
          this
        )
    );

  // SysID Methods
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  // Do we have coral?
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Intake/HasCoral?")
  public boolean hasCoral(){
      if(this.endEffectorIntake.getStatorCurrent().getValueAsDouble() > EndEffectorConstants.kEndEffectorCurrentSpike){
        return true;
      }else{
        return false;
      }
  }

  public BooleanSupplier hasCoralSupplier(){
    if(this.endEffectorIntake.getStatorCurrent().getValueAsDouble() > EndEffectorConstants.kEndEffectorCurrentSpike){
      return ()-> true;
    }else{
      return ()-> false;
    }
}

public BooleanSupplier endHP_CMD(){
  if(hasCoral() && isAtStowSetpoint()){
    return ()-> true;
  }else
    return ()-> false;

}

  //Is At Setpoint?
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Pivot/Position/IsAtSetpoint?")
  public BooleanSupplier isAtSetpoint(){
    return () -> Math.abs(getPivotPosition() - getPivotSetpoint()) < 0.05;
  }

  public boolean isAtStowSetpoint(){
    return Math.abs(getPivotPosition() - SetpointConstants.kStowEndEffectorSetpoint) < 0.05;
  }

  // Get Pivot Position
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Pivot/Position/PivotPosition")
  public double getPivotPosition() {
    return endEffectorPivot.getPosition().getValueAsDouble();
  }

  // Get Pivot Position Setpoint
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Pivot/Position/PivotSetpoint")
  public double getPivotSetpoint(){
    return m_motionRequest.Position;
  }



  // Get Pivot Velocity
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Pivot/PivotVelocity")
  public double getPivotVelocity() {
    return endEffectorPivot.get();
  }

  // Get Pivot Current
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Pivot/PivotCurrent")
  public double getPivotCurrent() {
    return endEffectorPivot.getSupplyCurrent().getValueAsDouble();
  }

  // Get Pivot Voltage
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Pivot/PivotVoltage")
  public double getPivotMotorVoltage(){
    return endEffectorPivot.getMotorVoltage().getValueAsDouble();
  }



  // Get Intake Velocity
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Intake/IntakeVelocity")
  public double getIntakeVelocity() {
    return endEffectorIntake.get();
  }

 // Get Intake Current
 @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Intake/IntakeCurrent")
 public double getIntakeSupplyCurrent() {
   return endEffectorIntake.getSupplyCurrent().getValueAsDouble();
 }

  // Get Intake Voltage
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Intake/IntakeVoltage")
  public double getIntakeVoltage() {
    return endEffectorIntake.getMotorVoltage().getValueAsDouble();
  }

  //Get EE Encoder Position
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Pivot/Position/EndEffectorEncoderPosition")
  public double getEndEffectorEncoderPivotPosition(){
    return endEffectorEncoder.getPosition().getValueAsDouble();
  }

  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/IntakeIntakeStatorCurrent")
  public double getIntakeStatorCurrent(){
    return endEffectorIntake.getStatorCurrent().getValueAsDouble();
  }


  @Override
  public void periodic() {




  // System.out.println(endEffectorIntake.getStatorCurrent().getValueAsDouble());


  





}
}
// Merge change



  





