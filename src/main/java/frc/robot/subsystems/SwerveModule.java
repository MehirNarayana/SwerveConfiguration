// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  TalonFX driveMotor;
  TalonFX steeringMotor;
  int dCanID;
  int tCanID;
  double offset;
  PIDController turningPid = new PIDController(1, 0, 0);
  PIDController drivePid = new PIDController(0, 0, 0);
  public SwerveModule(int driveMotorid, int turningMotorid, boolean driveMotorInverted, boolean turningMotorInverted, double offset) {
    driveMotor = new TalonFX(driveMotorid);
    steeringMotor = new TalonFX(turningMotorid);
    driveMotor.setInverted(driveMotorInverted);
    steeringMotor.setInverted(turningMotorInverted);
    turningPid.enableContinuousInput(-180, 180);
    this.offset = offset;
    SmartDashboard.putNumber("offset"+tCanID,getTurningPosition());
  }

  public double getDrivePosition() {
    StatusSignal<Double> pos = driveMotor.getPosition();
    return pos.getValue().doubleValue();
    
  }

  public double getTurningPosition() {
    StatusSignal<Double> pos = steeringMotor.getPosition();
    double currAngle = (pos.getValue().doubleValue()-offset)/2048/(150/7)*360;
    currAngle = Math.signum(currAngle)*(Math.abs(currAngle)%360);
    if (currAngle<0){
      currAngle = currAngle+360;
    }
    return currAngle;
  }

  public double getDriveVelocity() {
    StatusSignal<Double> driveVelocity = driveMotor.getVelocity();
    return driveVelocity.getValue().doubleValue();
  }

  public double getTurningVelocity() {
    StatusSignal<Double> steeringVelocity = steeringMotor.getVelocity();
    return steeringVelocity.getValue().doubleValue()/2048/(150/7)*360;
  }

 

  public void overideSpeed(double driveSpeed, double turningSpeed){
    driveMotor.set(driveSpeed);
    steeringMotor.set(turningSpeed);
  }

  public void stop(){
    driveMotor.set(0);
    steeringMotor.set(0);
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drive Position"+dCanID, getDrivePosition());
    SmartDashboard.putNumber("Turn Position"+tCanID, getTurningPosition());
    SmartDashboard.putNumber("Drive Position"+dCanID, getDrivePosition());
    SmartDashboard.putNumber("Turn velocity"+tCanID, getTurningVelocity());

    // This method will be called once per scheduler run
  }


}
