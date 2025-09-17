// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private TalonFX climberMotor;
  private TalonFXConfiguration climberConfigs;

  private CANcoder climberEncoder;
  private CANcoderConfiguration climberEncoderConfigs;
  

  public ClimberSubsystem() {

    climberMotor = new TalonFX(ClimberConstants.kClimberMotorID, "Drive CANivore");
    climberConfigs = new TalonFXConfiguration()
                          .withMotorOutput(new MotorOutputConfigs()
                                          .withNeutralMode(NeutralModeValue.Brake)
                                          .withInverted(InvertedValue.CounterClockwise_Positive))
                          .withFeedback(new FeedbackConfigs()
                                        .withFeedbackRemoteSensorID(ClimberConstants.kClimberEncoderID)
                                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder))
                          .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                                                      .withForwardSoftLimitEnable(true)
                                                      .withForwardSoftLimitThreshold(-0.23) //0.765
                                                      .withReverseSoftLimitEnable(true)
                                                      .withReverseSoftLimitThreshold(-0.515)) // 0.485
                          .withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
                                                      .withContinuousWrap(true));
    climberMotor.getConfigurator().apply(climberConfigs);

    climberEncoder = new CANcoder(ClimberConstants.kClimberEncoderID,"Drive CANivore");
    climberEncoderConfigs = new CANcoderConfiguration()
                              .withMagnetSensor(new MagnetSensorConfigs()
                                                .withMagnetOffset(ClimberConstants.kEncoderOffset)
                                                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                                                ); 
    climberEncoder.getConfigurator().apply(climberEncoderConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberUp(){
    climberMotor.set(ClimberConstants.kClimberSpeed);
  }

  public void climberDown(){
    climberMotor.set(-ClimberConstants.kClimberSpeed);
  }

  public void climberStop(){
    climberMotor.set(0);
  }


  // get Intake Velocity
  @AutoLogOutput(key  = "Subsystems/ClimberSubsystem/Motor/ClimberVelocity")
  public double getIntakeVelocity(){
    return climberMotor.get();
  }

  // get Intake Current
  @AutoLogOutput(key  = "Subsystems/ClimberSubsystem/Motor/ClimberCurrent")
  public double getIntakeCurrent(){
    return climberMotor.getSupplyCurrent().getValueAsDouble();
  }

  // get Intake Voltage
  @AutoLogOutput(key  = "Subsystems/ClimberSubsystem/Motor/CLimberVoltage")
  public double getIntakeMotorVoltage(){
    return climberMotor.getMotorVoltage().getValueAsDouble();
  }

  // // get Intake Temp
  // @AutoLogOutput(key  = "Subsystems/ClimberSubsystem/Motor/ClimberTemperature")
  // public double getIntakeMotorTemp(){
  //   return climberMotor.getDeviceTemp().getValueAsDouble();
  // }

  @AutoLogOutput(key  = "Subsystems/ClimberSubsystem/Position/ClimberPosition")
  public double getPivotPosition(){
    return climberMotor.getPosition().getValueAsDouble();
  }


}
