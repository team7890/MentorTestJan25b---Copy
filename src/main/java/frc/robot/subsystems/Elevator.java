// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// === REV Imports === \\
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// === Internal Imports === \\
import frc.robot.Constants;
import frc.robot.Constants.CANIds;

public class Elevator extends SubsystemBase {

  private SparkMax objElevator = new SparkMax(CANIds.iElevator, MotorType.kBrushless);

  private SparkMaxConfig objConfig = new SparkMaxConfig();

  /** Creates a new AlgaeTopRoller. */
  public Elevator() {
    objConfig.smartCurrentLimit(30);
    objConfig.idleMode(IdleMode.kBrake);
    objElevator.configure(objConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopElevator() {
    objElevator.stopMotor();
  }

  public void runElevator(double dSpeed) {
    objElevator.set(dSpeed);
  }
}
