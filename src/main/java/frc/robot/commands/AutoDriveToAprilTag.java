// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.UtilityFunctions;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

//dError = TX - dTarget
//dControl = dError * dKp

public class AutoDriveToAprilTag extends Command {

  private double dPropPartTX, dIntPartTX, dPropPartTY, dIntPartTY;
  
  private final CommandSwerveDrivetrain objSwerve;
  // private final LimelightHelpers objLimelight = new LimelightHelpers();
  private final double dMaxSpeed;
  private final double dMaxAngularRate;


  double dLimeTX, dErrorTX, dKpTX, dControlTX, dIntegralTX, dKiTX, dLimeTY, dErrorTY, dKpTY, dControlTY, dIntegralTY, dKiTY;
 
  private SwerveRequest.FieldCentric drive  = new SwerveRequest.FieldCentric()
  .withDeadband(0.0).withRotationalDeadband(0.0) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors;

  // = new SwerveRequest.FieldCentric()
  // .withDeadband(dMaxSpeed * 0.2).withRotationalDeadband(dMaxAngularRate * 0.15) // Add a 10% deadband
  // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


  /** Creates a new AutoDriveToAprilTag. */
  // public AutoDriveToAprilTag(CommandSwerveDrivetrain objSwerve_in, LimelightHelpers objLimelight_in) {
  public AutoDriveToAprilTag(CommandSwerveDrivetrain objSwerve_in, double dMaxSpeed_in, double dMaxAngularRate_in) {

    objSwerve = objSwerve_in;
    dMaxSpeed = dMaxSpeed_in;
    dMaxAngularRate = dMaxAngularRate_in;


    addRequirements(objSwerve);
    // Use addRequirements() here to declare subsystem dependencies.

    
  }
 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dIntegralTX = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // === LIMELIGHT ROTATIONAL === \\
    dLimeTX = LimelightHelpers.getTX("");
    dErrorTX = dLimeTX - 0.0;   //TODO: Change if needed
    dIntegralTX = dIntegralTX + dErrorTX;
    dIntegralTX = UtilityFunctions.limitVariable(-200.0, dIntegralTX, 200.0);
    dKiTX = 0.15 / 200.0;
    dKpTX = 0.45/25; //25 = farthest value from april tag
    dPropPartTX = -dErrorTX * dKpTX;

    dIntPartTX = -dIntegralTX * dKiTX;

    dControlTX = dPropPartTX + dIntPartTX;
      // === LIMELIGHT DISTANCE === \\
    dLimeTY = LimelightHelpers.getTY("");
    dErrorTY = dLimeTY - 0.0;
    dIntegralTY = dIntegralTY + dErrorTY;
    dIntegralTY = UtilityFunctions.limitVariable(-200.0, dIntegralTY, 200.0);
    dKiTY = 0.10 / 200.0;
    dKpTY = 0.45/25; //25 = farthest value from april tag
    dPropPartTY = -dErrorTY * dKpTY;

    dIntPartTY = -dIntegralTY * dKiTY;

    dControlTY = dPropPartTY + dIntPartTY;

    // dControl = -dError * dKp + -dIntegral * dKi;
    SmartDashboard.putNumber("Proportional Limelight", dPropPartTX);
    SmartDashboard.putNumber("Integral Limelight", dIntPartTX);
      
    objSwerve.setControl(
          drive.withVelocityX(-dControlTY).withVelocityY(0.0).withRotationalRate(dControlTX)
      );
    
    // System.out.println("AutoDriveToAprilTag");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objSwerve.setControl(
        drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0)
    );
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(objLimelight.getTargetAngle()) < 1.0;
    return false;
  }
}