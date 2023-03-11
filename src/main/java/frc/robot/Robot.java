// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;
//import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
//import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

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

  public double maxArm = 96.00; // encoder value at full extension for arm
  public double midArm = 50.00; // encoder value at mid extension for arm
  public double closedArm = 0; // encoder value at full retraction for arm 
  public double speedOut = 0.25;
  public double speedIn = -0.25;
  public boolean mArmGoToMAX = false;
  public boolean mArmGoToMID = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Power Distribution -- must be at CAN ID 1
    mPowerDistribution = new PowerDistribution(1, ModuleType.kRev);
    mPowerDistribution.clearStickyFaults();
    mPowerDistribution.setSwitchableChannel(false);

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

    // push values to dashboard here
    SmartDashboard.putNumber("[DT] LT-EncPos", mLeftEncoder.getPosition());
    SmartDashboard.putNumber("[DT] RT-EncPos", mRightEncoder.getPosition());
    SmartDashboard.putNumber("Arm", mArmEncoder.getPosition());   

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

    mArmGoToMAX = false;
    mArmGoToMID = false;

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    mRightEncoder.setPosition(0);
    mLeftEncoder.setPosition(0);
    //mArmEncoder.setPosition(0);

    mArm.stopMotor();
    mGrabber.stopMotor();
    mRobotDrive.arcadeDrive(0, 0);

    mArmGoToMAX = false;
    mArmGoToMID = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // Drive robot
     mSpeed = -mStick.getY() * ((mStick.getThrottle() * -0.5) + 0.5);
     mTwist = mStick.getTwist() * ((mStick.getThrottle() * -0.5) + 0.5);
     mRobotDrive.arcadeDrive(mSpeed, mTwist);   


    // Reset encoders 
    if (mStick.getRawButton(12)) {
      mRightEncoder.setPosition(0);
      mLeftEncoder.setPosition(0);
      mArmEncoder.setPosition(0);   
    }

    //Grabber control
    if (mXbox.getLeftBumper()) { //Signal CONE lights
      mGrabber.stopMotor();
      //light to YELLOW
    }
    else if (mXbox.getLeftTriggerAxis() == 1){ //Pickup CONE
      mGrabber.set(0.25);
    }
    else if (mXbox.getRightBumper()) { //Signal CUBE lights
      mGrabber.stopMotor();
      //light to PURPLE
    }
    else if (mXbox.getRightTriggerAxis() == 1){ //Pickup CUBE
      mGrabber.set(-0.25);
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
      if (mArmEncoder.getPosition() < maxArm) {
        mArm.set(speedOut);
      }
      else {
        mArm.stopMotor();
      }

      if (mArmEncoder.getPosition() >= maxArm) {
        mArmGoToMAX = false;
      }
    }
    else if ((mXbox.getPOV()==90) || (mXbox.getPOV()==270) || mArmGoToMID) {  //Extend arm to MID
      mArmGoToMID = true;
      if (mArmEncoder.getPosition() < midArm) {
        mArm.set(speedOut);
      }
      else {
        mArm.stopMotor();
      }
      if (mArmEncoder.getPosition() >= midArm) {
        mArmGoToMID = false;
      }
    }
    else if ((mXbox.getPOV()==180)) {  // Retract Arm fully
      if (mArmEncoder.getPosition() > closedArm) {
        mArm.set(speedIn);
      }
      else {
        mArm.stopMotor();
      }
      mArmGoToMID = false;
      mArmGoToMAX = false;
    }
    else if (mXbox.getYButtonPressed()) { // Move arm out
      mArm.set(speedOut);
    }
    else if (mXbox.getAButtonPressed()) { // Move arm in
      mArm.set(speedIn);
    }
    else {
      mArm.stopMotor();
      mArmGoToMID = false;
      mArmGoToMAX = false;
    }

  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    mRightEncoder.setPosition(0);
    mLeftEncoder.setPosition(0);
    mArmEncoder.setPosition(0);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    mRightEncoder.setPosition(0);
    mLeftEncoder.setPosition(0);
    mArmEncoder.setPosition(0);
 
  }
}
