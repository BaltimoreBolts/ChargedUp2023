// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public AHRS mGyro;

  public CANSparkMax mLeftDriveMotor1;
  public CANSparkMax mRightDriveMotor1;
  public CANSparkMax mLeftDriveMotor2;
  public CANSparkMax mRightDriveMotor2;
  public DifferentialDrive mRobotDrive;
  public MotorControllerGroup mLeftMotors;
  public MotorControllerGroup mRightMotors;
  public RelativeEncoder mLeftEncoder;
  public RelativeEncoder mRightEncoder;

  public PowerDistribution mPowerDistribution;

  public CANSparkMax mArm;
  public CANSparkMax mGrabber;
  public RelativeEncoder mArmEncoder;

  public Joystick mStick;
  public XboxController mXbox;
  public double mSpeed = 0.0;
  public double mTwist = 0.0;

  public double wheelDia = 6.0; // inches
  public double gearRatio = 8.45; // Toughbox Mini
  public double rWidth = 2; // robot width in inches

  public double maxArm = -40; // encoder value at full extension for arm
  public double midArm = -25; // encoder value at mid extension for arm
  public double closedArm = 1; // encoder value at full retraction for arm 
  public double speedOut = -0.25;
  public double speedIn = 0.25;
  public boolean mArmGoToMAX = false;
  public boolean mArmGoToMID = false;
  public boolean mArmGoToHOME = false;
  public boolean setMotorsToBrake = false;

  public double autonStartTime;
  public double autonWaitTime = 0; // seconds to wait
  public double autonCurrentTime;
  public double autonGPreleaseTime; 
  public double autonFinalPos = -160; //-180  inches to drive backwards
  public boolean autoArmExtend = false;
  public boolean autoGPrelease = false;
  public boolean autoArmRetract = false;
  public boolean autoMove = false;

  public String desiredColor;
  public double matchTimer;
  public double mCurrentAngle;

  public UsbCamera fishEye;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Open USB Camera
    fishEye = CameraServer.startAutomaticCapture();
    fishEye.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);
    fishEye.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    // Power Distribution -- must be at CAN ID 1
    mPowerDistribution = new PowerDistribution(1, ModuleType.kRev);
    mPowerDistribution.clearStickyFaults();
    mPowerDistribution.setSwitchableChannel(false);

    //navX
    try {
      mGyro = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
    }
    
    // Drive Motors
    mLeftDriveMotor1 = new CANSparkMax(5, MotorType.kBrushless);
    mLeftDriveMotor2 = new CANSparkMax(2, MotorType.kBrushless);
    mRightDriveMotor1 = new CANSparkMax(4, MotorType.kBrushless);
    mRightDriveMotor2 = new CANSparkMax(3, MotorType.kBrushless);
    mLeftMotors = new MotorControllerGroup(mLeftDriveMotor1, mLeftDriveMotor2);
    mRightMotors = new MotorControllerGroup(mRightDriveMotor1, mRightDriveMotor2);

    mLeftDriveMotor1.restoreFactoryDefaults();
    mLeftDriveMotor2.restoreFactoryDefaults();
    mRightDriveMotor1.restoreFactoryDefaults();
    mRightDriveMotor2.restoreFactoryDefaults();
    mLeftDriveMotor1.setSmartCurrentLimit(40);
    mLeftDriveMotor2.setSmartCurrentLimit(40);
    mRightDriveMotor1.setSmartCurrentLimit(40);
    mRightDriveMotor2.setSmartCurrentLimit(40);
    mLeftDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mLeftDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mRightDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mRightDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

    mLeftDriveMotor1.follow(mLeftDriveMotor2);
    mRightDriveMotor1.follow(mRightDriveMotor2);

    mLeftEncoder = mLeftDriveMotor1.getEncoder();
    mRightEncoder = mRightDriveMotor1.getEncoder();

    // Convert raw encoder units to inches using PIxdia/gearRatio
    mLeftEncoder.setPositionConversionFactor((Math.PI * wheelDia) / gearRatio);
    mRightEncoder.setPositionConversionFactor((Math.PI * wheelDia) / gearRatio);

    mRightDriveMotor1.setInverted(true);
    mRightDriveMotor2.setInverted(true);

    mLeftDriveMotor1.burnFlash();
    mLeftDriveMotor2.burnFlash();
    mRightDriveMotor1.burnFlash();
    mRightDriveMotor2.burnFlash();

    mRobotDrive = new DifferentialDrive(mLeftMotors, mRightMotors);
    setMotorsToBrake = false;

    // GPM Motors
    mArm = new CANSparkMax(6, MotorType.kBrushless);
    mGrabber = new CANSparkMax(7, MotorType.kBrushless);
    
    mArm.restoreFactoryDefaults();
    mGrabber.restoreFactoryDefaults();
    mArm.setSmartCurrentLimit(40);
    mGrabber.setSmartCurrentLimit(40);
    mArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mGrabber.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mArmEncoder = mArm.getEncoder();

    mArm.burnFlash();
    mGrabber.burnFlash();

    mStick = new Joystick(0);
    mXbox = new XboxController(1);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    mCurrentAngle = mGyro.getAngle();
    SmartDashboard.putNumber("Gyro", mCurrentAngle);

    // push values to dashboard here
    SmartDashboard.putNumber("[DT] LT-EncPos", mLeftEncoder.getPosition());
    SmartDashboard.putNumber("[DT] RT-EncPos", mRightEncoder.getPosition());
    SmartDashboard.putNumber("Arm", mArmEncoder.getPosition());   
    SmartDashboard.putBoolean("maxArm", mArmGoToMAX);
    SmartDashboard.putBoolean("midArm", mArmGoToMID);
    SmartDashboard.putBoolean("homeArm", mArmGoToHOME);

    matchTimer = DriverStation.getMatchTime();
    SmartDashboard.putNumber("MatchTime", matchTimer);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

    mRightEncoder.setPosition(0);
    mLeftEncoder.setPosition(0);
    mArmEncoder.setPosition(0);
    autonStartTime = Timer.getFPGATimestamp();

    mArmGoToMAX = false;
    mArmGoToMID = false;
    mArmGoToHOME = false;

    autoArmExtend = false;
    autoGPrelease = false;
    autoArmRetract = false;
    autoMove = false;

    mGyro.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    autonCurrentTime = Timer.getFPGATimestamp();
    autonWaitTime = 0;

 // wait x time
 if ((autonCurrentTime - autonStartTime) >= autonWaitTime) {
  /*
  * arm extend out
  * GP release
  * arm retract
  * move back
  * 
  */

    if (!autoArmExtend){
      if ((mArmEncoder.getPosition() > maxArm) && (autonCurrentTime - autonStartTime <= 3)) {
        mArm.set(speedOut);
      }
      else {
        mArm.stopMotor();
        autoArmExtend = true;
        autonGPreleaseTime= Timer.getFPGATimestamp();
      }
    }

    if (!autoGPrelease && autoArmExtend) {
        mGrabber.set(0.35);
        if ((autonCurrentTime - autonGPreleaseTime) >= 1){
          mGrabber.stopMotor();
          autoGPrelease = true;
        }
    }

    if (!autoArmRetract && autoGPrelease) {
      if (mArmEncoder.getPosition() < closedArm) {
        mArm.set(speedIn);
      }
      else {
        mArm.stopMotor();
        autoArmRetract = true;
      }
    }

    if (!autoMove && autoArmRetract) {
       // move at least X" backwards (negative position)
      if (mLeftEncoder.getPosition() > autonFinalPos) {
        mRobotDrive.arcadeDrive(-0.5, 0);
      } else {
        mRobotDrive.arcadeDrive(0, 0);
      }
    } else {
     mRobotDrive.arcadeDrive(0, 0);
    }
 
    } // not past wait time
    else {
      mRobotDrive.arcadeDrive(0, 0);
    }
 
  } //end routine

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    mRightEncoder.setPosition(0);
    mLeftEncoder.setPosition(0);
    mArmEncoder.setPosition(0);

    mArm.stopMotor();
    mGrabber.stopMotor();
    mRobotDrive.arcadeDrive(0, 0);

    mArmGoToMAX = false;
    mArmGoToMID = false;
    mArmGoToHOME = false;
  
    mLeftDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mLeftDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mRightDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mRightDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);

    mLeftDriveMotor1.burnFlash();
    mLeftDriveMotor2.burnFlash();
    mRightDriveMotor1.burnFlash();
    mRightDriveMotor2.burnFlash();

    setMotorsToBrake = false;
    mGyro.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


    // Switch drive to BRAKE in end game
    
    if ((matchTimer < 30) && !setMotorsToBrake) {
      mLeftDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
      mLeftDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
      mRightDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
      mRightDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);  

      mLeftDriveMotor1.burnFlash();
      mLeftDriveMotor2.burnFlash();
      mRightDriveMotor1.burnFlash();
      mRightDriveMotor2.burnFlash();

      setMotorsToBrake = true;
    }
    
    // Drive robot
     mSpeed = -mStick.getY() * ((mStick.getThrottle() * -0.5) + 0.5);
     mTwist = mStick.getTwist() * ((mStick.getThrottle() * -0.5) + 0.5);
     mRobotDrive.arcadeDrive(mSpeed, -mTwist);   

    // Reset encoders and gyro
    if (mStick.getRawButton(12)) {
      mRightEncoder.setPosition(0);
      mLeftEncoder.setPosition(0);
      mArmEncoder.setPosition(0);   
      mGyro.reset();
    }

    //Grabber control
    if (mXbox.getLeftBumper()) { //Signal CONE lights
      //light to YELLOW
    }
    else if (mXbox.getLeftTriggerAxis() == 1){ //Pickup CONE
      mGrabber.set(-0.45);
    }
    else if (mXbox.getRightBumper()) { //Signal CUBE lights
      //light to PURPLE
    }
    else if (mXbox.getRightTriggerAxis() == 1){ //Pickup CUBE
      mGrabber.set(0.35);
    }    
    else {
      mGrabber.stopMotor();
    }

    //Arm control
    /*
    Auto position: 
      Full Extension = DPad UP
      Mid Extension = DPad LEFT or RIGHT
      Full Retract = DPad DOWN  (mXbox.getPOV() == 180)
      
    Control
      Y button = arm out (hold)
      A button = arm in (hold)
    */

    if ((mXbox.getPOV()==0) || mArmGoToMAX){  //Extend arm to MAX
      mArmGoToMAX = true;
      if (mArmEncoder.getPosition() > maxArm) {
        mArm.set(speedOut);
      }
      else {
        mArm.stopMotor();
      }

      if (mArmEncoder.getPosition() <= (maxArm)) {
        mArmGoToMAX = false;
      }
    }
    else if ((mXbox.getPOV()==90) || (mXbox.getPOV()==270) || mArmGoToMID) {  //Extend arm to MID
      mArmGoToMID = true;
      if (mArmEncoder.getPosition() > midArm+5) {
        mArm.set(speedOut);
      }
      else if (mArmEncoder.getPosition() < (midArm-5)) {
        mArm.set(speedIn);
      }
      else {
        mArm.stopMotor();
      }
      if ((mArmEncoder.getPosition() <= (midArm+5)) && (mArmEncoder.getPosition() >= (midArm-5))) {
        mArmGoToMID = false;
      }
    }
    else if ((mXbox.getPOV()==180) || mArmGoToHOME) {  // Retract Arm fully
      if (mArmEncoder.getPosition() < closedArm) {
        mArm.set(speedIn);
      }
      else {
        mArm.stopMotor();
        mArmGoToHOME = false;
      }
      mArmGoToMID = false;
      mArmGoToMAX = false;
      mArmGoToHOME = false;
    }
    else if (mXbox.getYButton()) { // Move arm out
      mArm.set(speedOut);
    }
    else if (mXbox.getAButton()) { // Move arm in
      mArm.set(speedIn);
    }
    else {
      mArm.stopMotor();
      mArmGoToMID = false;
      mArmGoToMAX = false;
      mArmGoToHOME = false;
    }

  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    mRightEncoder.setPosition(0);
    mLeftEncoder.setPosition(0);
    mArmEncoder.setPosition(0);
    mArmGoToMID = false;
    mArmGoToMAX = false;
    mArmGoToHOME = false;
    autoArmExtend = false;
    autoGPrelease = false;
    autoArmRetract = false;
    autoMove = false;
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    mRightEncoder.setPosition(0);
    mLeftEncoder.setPosition(0);
    mArmEncoder.setPosition(0);
    mArmGoToMID = false;
    mArmGoToMAX = false;
    mArmGoToHOME = false;
    autoArmExtend = false;
    autoGPrelease = false;
    autoArmRetract = false;
    autoMove = false;
 
  }
}
