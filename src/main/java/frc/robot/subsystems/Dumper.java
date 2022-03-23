// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Dumper extends SubsystemBase {
  /** Creates a new Dumper. */
  private WPI_VictorSPX m_dumperIntakeOuttake;
  private CANSparkMax m_dumperArm;
  private RelativeEncoder m_armEncoder;
  private int position;
  private DigitalInput m_topLimitSwitch;
  private DigitalInput m_bottomLimitSwitch;

  public Dumper() {
    m_dumperIntakeOuttake = new WPI_VictorSPX(Constants.dumperPort);
    m_dumperIntakeOuttake.setNeutralMode(NeutralMode.Brake);
    m_dumperArm = new CANSparkMax(Constants.dumperArmPort, MotorType.kBrushless);
    m_armEncoder = m_dumperArm.getEncoder();
    m_topLimitSwitch = new DigitalInput(Constants.topLimitSwitch);
    m_bottomLimitSwitch = new DigitalInput(Constants.bottomLimitSwitch);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeOuttake(double dumperVoltage) {
    m_dumperIntakeOuttake.set(ControlMode.PercentOutput, dumperVoltage);
  }

  public void passiveSpin(double dumperVoltage) {
    m_dumperIntakeOuttake.set(dumperVoltage);
  }

  public void moveArm(double dumperUpSpeed){
    m_dumperArm.setVoltage(dumperUpSpeed);
  }

  public void stopIntakeOuttake() {
    m_dumperIntakeOuttake.stopMotor();
  }

  public double getPosition() {
    return m_armEncoder.getPosition();
  }

  public void stopArm() {
    m_dumperArm.stopMotor();
  }

  public void isDown() {
    position = -1;
  }

  public void isUp() {
    position = 1;
  }

  public void isMiddle() {
    position = 0;
  }

  public void checkArmPosition() {
    if (m_topLimitSwitch.get()) {
      isUp();
    }
    else if (m_bottomLimitSwitch.get()) {
      isDown();
    }
    else {
      isMiddle();
    }
  }

  public int getArmPosition() {
    return position;
  }

  public DigitalInput getTopSwitch() {
    return m_topLimitSwitch;
  }

  public DigitalInput getBottomSwitch() {
    return m_bottomLimitSwitch;
  }

  public void setToCoast() {
    m_dumperArm.setIdleMode(IdleMode.kCoast);
  }

  public void setToBrake(){
    m_dumperArm.setIdleMode(IdleMode.kBrake);
  }
}