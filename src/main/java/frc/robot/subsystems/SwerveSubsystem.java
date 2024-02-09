package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(0, 1, true, true,0,0);

    private final SwerveModule frontRight = new SwerveModule(0, 1, true, true,0,0);

    private final SwerveModule backLeft = new SwerveModule(0, 1, true, true,0,0);

    private final SwerveModule backRight = new SwerveModule(0, 1, true, true,0,0);

    private final AHRS gyro = new AHRS();
    double kWheelBase = 0;
    double kTrackWidth = 0;
   

  
    

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    

    public void zeroHeading() {
      gyro.reset();
    }

    public double getHeading() {
      return gyro.getYaw();
    }

    public Rotation2d getRotation2d() {
      return Rotation2d.fromDegrees(getHeading());
    }

  

    public void overideSpeed(double driveSpeed, double turningSpeed){
      frontLeft.overideSpeed(driveSpeed, turningSpeed);
      frontRight.overideSpeed(driveSpeed, turningSpeed);
      backLeft.overideSpeed(driveSpeed, turningSpeed);
      backRight.overideSpeed(driveSpeed, turningSpeed);
    }

    

    @Override
    public void periodic() {
    
     SmartDashboard.putNumber("yaw", getHeading());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

   
    

    

    

    

    
    

}