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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(0, 1, true, true);

    private final SwerveModule frontRight = new SwerveModule(0, 1, true, true);

    private final SwerveModule backLeft = new SwerveModule(0, 1, true, true);

    private final SwerveModule backRight = new SwerveModule(0, 1, true, true);

    private final AHRS gyro = new AHRS();
    double kWheelBase = 0;
    double kTrackWidth = 0;
    public final SwerveDriveKinematics kDriveKinematics = Constants.SwerveConstants.kDriveKinematics;

  
    private final SwerveDriveOdometry odometer;

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        

        
        
        SwerveModulePosition[] modulePositions = getModulePositions();
        odometer = new SwerveDriveOdometry(kDriveKinematics,
        new Rotation2d(0), modulePositions);


        


    }

    public SwerveModulePosition[] getModulePositions(){
      SwerveModulePosition[] modulePositions = {frontLeft.getPostion(), frontRight.getPostion(), 
        backLeft.getPostion(), backRight.getPostion()};
      return modulePositions;
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

    public Pose2d getPose() {
      return odometer.getPoseMeters();
    }

    public void overideSpeed(double driveSpeed, double turningSpeed){
      frontLeft.overideSpeed(driveSpeed, turningSpeed);
      frontRight.overideSpeed(driveSpeed, turningSpeed);
      backLeft.overideSpeed(driveSpeed, turningSpeed);
      backRight.overideSpeed(driveSpeed, turningSpeed);
    }

    

    @Override
    public void periodic() {
      SwerveModulePosition[] modulePositions = getModulePositions();
      odometer.update(getRotation2d(), modulePositions);
      SmartDashboard.putNumber("Robot Heading", getHeading());
      SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

   
    

    

    

    

    
    

}