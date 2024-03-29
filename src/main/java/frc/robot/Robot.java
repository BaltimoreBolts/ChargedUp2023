// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

  //public DigitalInput mSwitch;
  //public boolean mAutonSwitch;

  //public DigitalInput mSwitch2;
  //public boolean mAutonSwitch2;
  
  public static final String kConeAuto = "kConeAuto";
  public static final String kConeParkAuto = "kConeParkAuto";
  public static final String kBlue2Auto = "kBlue2Auto";
  public static final String kRed2Auto = "kRed2Auto";
  public static final String kConeStayStillAuto = "kConeStayStillAuto";
  public static final String kEngageAuto = "kEngageAuto";
  public String m_autoSelected;
  public final SendableChooser<String> m_chooser = new SendableChooser <> ();

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

  public CANSparkMax mIntakeExt;
  public CANSparkMax mIntakeSpin;
  public CANSparkMax mTunnelSpin;
  public RelativeEncoder mIntakeExtEncoder;

  public Joystick mStick;
  public XboxController mXbox;
  public double mSpeed = 0.0;
  public double mTwist = 0.0;

  public double wheelDia = 6.0; // inches
  public double gearRatio = 8.45; // Toughbox Mini
  public double rWidth = 2; // robot width in inches

  public double maxArm = -45; // encoder value at full extension for arm
  public double midArm = -16; // encoder value at mid extension for arm
  public double closedArm = 1; // encoder value at full retraction for arm 
  public double speedOut = -0.25;
  public double speedIn = 0.25;
  public double armStartTime;
  public boolean mArmGoToMAX = false;
  public boolean mArmGoToMID = false;
  public boolean mArmGoToHOME = false;
  public boolean setMotorsToBrake = false;

  public double intakeStartTime;
  public boolean intaking;
  public double intakeOut = 10; // encoder value at full extension for arm (guess) --> find real val
  public double closedIntake = 0;
  public boolean intakeExtend = false;
  public boolean intakeIn = false;

  public double autonStartTime;
  public double autonWaitTime = 0; // seconds to wait
  public double autonCurrentTime;
  public double autonGPreleaseTime; 
  public double autonFinalPos = -160; //-160  inches to drive backwards
  public double autonFinalPark = -85; // distance to park or posibly engage
  public double autoEngageHalfPoint = 85;
  public double autonCubePos = -165; //-180  inches to drive backwards
  public double autoCubeNodePos = 185; //189  inches to cube node
  public double autonIntoCubePos = -25;
  public double gryoCubeAngleRED = -35; // turn to face cube
  public double gryoCubeNodeAngleRED = 35; // turn to face cube node
  public double gryoCubeAngleBLUE = 35; // turn to face cube
  public double gryoCubeNodeAngleBLUE = -35; // turn to face cube node
  public boolean autoArmExtend = false;
  public boolean autoGPrelease = false;
  public boolean autoArmRetract = false;
  public boolean autoMove = false;
  public boolean autonIntakeExtend = false;
  public boolean autoCubeTurn = false;
  public boolean autoIntoCube = false;
  public boolean autoCubeNodeTurn = false;
  public boolean autoAtCubeNode = false;
  public double autonGPreleaseTime2;
  public boolean autoGPrelease2 = false;
  public boolean autoIntakeIn = false;
  public boolean autoArmExtend2 = false;
  public boolean encoderReset = false; 
  public boolean AutocubeInMore = false;
  public boolean autoNotTipped = false;
  public double autonArmOutTime;
  public double autoMoreCubeInTime;

  //public String desiredColor;
  public String autonRoutine;
  public double matchTimer;
  public double mCurrentAngle;
  public double mYaw;
  public double mPitch;
  public double mRoll;

  public double direction;
  public AddressableLED mLED;
  public AddressableLEDBuffer mLEDbuffer;

  public UsbCamera fishEye;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    //LED logic
    /*
    mLED = new AddressableLED(0);
    mLEDbuffer = new AddressableLEDBuffer(15);
    mLED.setLength(mLEDbuffer.getLength());
    mLED.setData(mLEDbuffer);
    mLED.start();
    */
    
    m_chooser.setDefaultOption("Default Cone-Auto", kConeAuto);
    m_chooser.addOption("Cone-Park-Auto", kConeParkAuto);
    m_chooser.addOption("Blue-2-Auto", kBlue2Auto);
    m_chooser.addOption("Red-2-Auto", kRed2Auto);
    m_chooser.addOption("Cone-Stay-Still-Auto", kConeStayStillAuto);
    m_chooser.addOption("Engage-Cone", kEngageAuto); 
    SmartDashboard.putData("Auto",m_chooser);

    // Open USB Camera
    fishEye = CameraServer.startAutomaticCapture();
    fishEye.setVideoMode(PixelFormat.kYUYV, 320, 240, 15);
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
    mRobotDrive.setSafetyEnabled(false);

    setMotorsToBrake = false;

    // Intake And Tunnel
    mIntakeExt = new CANSparkMax(9, MotorType.kBrushless);
    mIntakeSpin = new CANSparkMax(8, MotorType.kBrushless);
    mTunnelSpin = new CANSparkMax(10, MotorType.kBrushless);

    mIntakeExt.restoreFactoryDefaults();
    mIntakeSpin.restoreFactoryDefaults();
    mTunnelSpin.restoreFactoryDefaults();
    mIntakeExt.setSmartCurrentLimit(40);
    mIntakeSpin.setSmartCurrentLimit(40);
    mTunnelSpin.setSmartCurrentLimit(40);
    mIntakeExt.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mIntakeSpin.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mTunnelSpin.setIdleMode(CANSparkMax.IdleMode.kCoast);

    mIntakeExtEncoder = mIntakeExt.getEncoder();

    mIntakeExt.burnFlash();
    mIntakeSpin.burnFlash();
    mTunnelSpin.burnFlash();

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

    /*
    for (var i = 0; i < mLEDbuffer.getLength(); i++) {
      //sets the specified LED to the RGB values for blue
      mLEDbuffer.setRGB(i, 255, 0, 0);
      //mLEDbuffer.setHSV(i, 160, 240, 120);
    }

    mLED.setData(mLEDbuffer);
    */

    if (DriverStation.isAutonomous()) {
      mCurrentAngle = mGyro.getAngle();
      mRoll = mGyro.getRoll();
      //mYaw = mGyro.getYaw();
      //mPitch = mGyro.getPitch();
    }
    
    SmartDashboard.putNumber("Gyro", mCurrentAngle);
    //SmartDashboard.putNumber("Yaw", mYaw);
    //SmartDashboard.putNumber("Pitch", mPitch);
    SmartDashboard.putNumber("Roll", mRoll);
    
    m_autoSelected = m_chooser.getSelected();

    // push values to dashboard here
    SmartDashboard.putNumber("Intake", mIntakeExtEncoder.getPosition());  
    SmartDashboard.putNumber("[DT] LT-EncPos", mLeftEncoder.getPosition());
    SmartDashboard.putNumber("[DT] RT-EncPos", mRightEncoder.getPosition());
    SmartDashboard.putNumber("Arm", mArmEncoder.getPosition());   
    SmartDashboard.putBoolean("maxArm", mArmGoToMAX);
    SmartDashboard.putBoolean("midArm", mArmGoToMID);
    SmartDashboard.putBoolean("homeArm", mArmGoToHOME);
    SmartDashboard.putString("Selected Auto", m_autoSelected);
    SmartDashboard.putBoolean("autoArmExtend", autoArmExtend);
    SmartDashboard.putBoolean("autoGPrelease", autoGPrelease);
    SmartDashboard.putBoolean("autoArmRetract", autoArmRetract);
    SmartDashboard.putBoolean("autonIntakeExtend", autonIntakeExtend);
    SmartDashboard.putBoolean("autoMove", autoMove);
    SmartDashboard.putBoolean("autoCubeTurn", autoCubeTurn);
    SmartDashboard.putBoolean("Encoder Reset", encoderReset);
    SmartDashboard.putBoolean("autoIntoCube", autoIntoCube);
    SmartDashboard.putBoolean("autoCubeNodeTurn", autoCubeNodeTurn);
    SmartDashboard.putBoolean("autoIntakeIn", autoIntakeIn);
    SmartDashboard.putBoolean("AutocubeInMore", AutocubeInMore);
    SmartDashboard.putBoolean("autoArmExtend2", autoArmExtend2);
    SmartDashboard.putBoolean("autoAtCubeNode", autoAtCubeNode);
    SmartDashboard.putBoolean("autoGPrelease2", autoGPrelease2);
    
    //matchTimer = DriverStation.getMatchTime();
    //SmartDashboard.putNumber("MatchTime", matchTimer);

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

    m_autoSelected = m_chooser.getSelected();

    mRightEncoder.setPosition(0);
    mLeftEncoder.setPosition(0);
    mArmEncoder.setPosition(0);
    mIntakeExtEncoder.setPosition(0);
    autonStartTime = Timer.getFPGATimestamp();

    mArmGoToMAX = false;
    mArmGoToMID = false;
    mArmGoToHOME = false;

    autoArmExtend = false;
    autoArmExtend2 = false;
    autoGPrelease = false;
    autoArmRetract = false;
    autoMove = false;
    autonIntakeExtend = false;
    autoCubeTurn = false;
    autoIntoCube = false;
    autoCubeNodeTurn = false;
    autoAtCubeNode = false;
    autoGPrelease2 = false;
    autoIntakeIn = false;
    encoderReset = false;
    AutocubeInMore = false;
    autoNotTipped = false;

    intakeExtend = false;
    intakeIn = false;

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
    if ((m_autoSelected == kConeAuto) || (m_autoSelected == kConeParkAuto) || (m_autoSelected == kConeStayStillAuto) || (m_autoSelected == kEngageAuto)) { 
      // standard auto (cone and leave community) & park auto
      if (!autoArmExtend){
        if ((mArmEncoder.getPosition() > maxArm) && (autonCurrentTime - autonStartTime <= 2.25)) { //arm out with 3 second breakout
          mArm.set(speedOut);
          mGrabber.set(-0.1); //cone
          //mGrabber.set(0.1); //cube
        }
        else {
          mArm.stopMotor();
          mGrabber.stopMotor();
          autoArmExtend = true;
          autonGPreleaseTime= Timer.getFPGATimestamp();
        }
      }

      if (!autoGPrelease && autoArmExtend) {
          mGrabber.set(0.35); //cone
          //mGrabber.set(-0.35); //cube
          if ((autonCurrentTime - autonGPreleaseTime) >= .5){
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

      if (!autoMove && autoGPrelease && (m_autoSelected == kConeAuto)) {
        // move at least X" backwards (negative position)
        if (mLeftEncoder.getPosition() > autonFinalPos) { // drive back for normal mobility (160")
          mRobotDrive.arcadeDrive(-0.5, 0);
        } else {
          mRobotDrive.arcadeDrive(0, 0);
        }
      } 
      else if (!autoMove && autoGPrelease && (m_autoSelected == kConeParkAuto)) {
        // move at least X" backwards (negative position)
        if (mLeftEncoder.getPosition() > autonFinalPark+10) { // drive back for dock (all at 45% except last 10") 85" total
          mRobotDrive.arcadeDrive(-0.45, 0);
        }
        else if (mLeftEncoder.getPosition() > autonFinalPark) {
          mRobotDrive.arcadeDrive(-0.25, 0);
        } else {
          mRobotDrive.arcadeDrive(0, 0);
        }
      }
      else if (!autoGPrelease2 && autoGPrelease && (m_autoSelected == kEngageAuto)) {
        if (-170 <= mCurrentAngle){ //turns 180 degrees CCW (we want 40 from the overshoot)
          mRobotDrive.arcadeDrive(0, 0.35); //val is opposite for all turns due to flipped stick
        } else {
          mRobotDrive.arcadeDrive(0, 0);
          mLeftEncoder.setPosition(0);
          mRightEncoder.setPosition(0);
          autoMoreCubeInTime = Timer.getFPGATimestamp();
          autoGPrelease2 = true;
        }
      } else if (!autoMove && autoGPrelease2 && (m_autoSelected == kEngageAuto)) {
        if (autonCurrentTime - autoMoreCubeInTime < 0.5) {
          mRobotDrive.arcadeDrive(0, 0);
        } else if (mLeftEncoder.getPosition() < autoEngageHalfPoint) { // drive back for dock (all at 45% except last 10") 85" total
          mRobotDrive.arcadeDrive(0.4, 0);
        } else if ((mRoll < -15) && (autoNotTipped == false)){
          mRobotDrive.arcadeDrive(-0.30, 0);
        } else if (Math.abs(mRoll) > 3) {
          //autoGPrelease2 = true;
          autoNotTipped = true;
          direction = -(mRoll/Math.abs(mRoll));
          mRobotDrive.arcadeDrive(.32*direction, 0);
        } else {
          //waitTime = Timer.getFPGATimestamp();
          mRobotDrive.arcadeDrive(0, 0);
        }
      }
      else {
      mRobotDrive.arcadeDrive(0, 0); // mid auto when we don't want to move - kConeStayStillAuto
      }

      //pull in arm during balancing; using the autoArmExtend2 (no longer used)
  /*     if (autoGPrelease2 && !autoArmExtend2) {
        if (mArmEncoder.getPosition() < closedArm) {
          mArm.set(speedIn);
        }
        else {
          mArm.stopMotor();
          autoArmExtend2 = true;
        }
      } */

    }
    
    else { // super auto

      if (!autoArmExtend){
        if ((mArmEncoder.getPosition() > maxArm) && (autonCurrentTime - autonStartTime <= 2.25)) { //arm out
          mArm.set(speedOut);
          mGrabber.set(-0.1); //keeps cone in
          mRobotDrive.arcadeDrive(0, 0);
        }
        else {
          mRobotDrive.arcadeDrive(0, 0);
          mArm.stopMotor();
          mGrabber.stopMotor();
          autoArmExtend = true;
          autonGPreleaseTime= Timer.getFPGATimestamp();
        }
      }

      if (!autoGPrelease && autoArmExtend) {
          mGrabber.set(0.35);
          mRobotDrive.arcadeDrive(0, 0);
          if ((autonCurrentTime - autonGPreleaseTime) >= 0.4){ //.75 seconds of shooting out cone
            mGrabber.stopMotor();
            autoGPrelease = true;
          }
      }

      if (!autoMove && autoGPrelease) { //moving towards cube --> arm in during movement, then intake out
        if (mLeftEncoder.getPosition() > autonCubePos) { //moving back 175"
          mRobotDrive.arcadeDrive(-0.5, 0);
        } else {
          mRobotDrive.arcadeDrive(0, 0);
          autoMove = true; //condition to breakout of the whole bit and move onto the next
        }
        if (!autoArmRetract && autoGPrelease) { //arm in starts when movement does
          if (mArmEncoder.getPosition() < closedArm) {
            mArm.set(speedIn);
          }
          else {
            mArm.stopMotor();
            autoArmRetract = true;
          }
        }
        else if (!autonIntakeExtend && autoArmRetract) { //intake in starts when the arm is retracted
          if (mIntakeExtEncoder.getPosition() < (intakeOut)) {
            mIntakeExt.set(.25);
          }
          else {
            mIntakeExt.stopMotor();
            autonIntakeExtend = true;
          }
        }
      }
      

      if (!autoCubeTurn && autoMove && (m_autoSelected == kRed2Auto)) {
        // move at least X" backwards (negative position)
        if (gryoCubeAngleRED <= mCurrentAngle){ //turns 35 degrees CCW (we want 40 from the overshoot)
          mRobotDrive.arcadeDrive(0, 0.35); //val is opposite for all turns due to flipped stick
        } else {
          mRobotDrive.arcadeDrive(0, 0);
          mLeftEncoder.setPosition(0);
          mRightEncoder.setPosition(0);
          autoCubeTurn = true;
        }
      }

      if (!autoCubeTurn && autoMove && (m_autoSelected == kBlue2Auto)) {
        if (gryoCubeAngleBLUE >= mCurrentAngle){ //turns 35 degrees CW (we want 40 from the overshoot)
          mRobotDrive.arcadeDrive(0, -0.35);
        } else {
          mRobotDrive.arcadeDrive(0, 0);
          mLeftEncoder.setPosition(0);
          mRightEncoder.setPosition(0);
          autoCubeTurn = true;
        }
      }

      if(!encoderReset && autoCubeTurn && (mLeftEncoder.getPosition()*mLeftEncoder.getPosition() <= 2)){ //bit that ensures drive encoders are reset before proceeding
        encoderReset = true;
        mGyro.reset();
      }
      if(!encoderReset && autoCubeTurn){ //keep drive happy 
        mRobotDrive.arcadeDrive(0, 0);
      }

      if (!autoIntoCube && encoderReset) {
        if (mLeftEncoder.getPosition() > autonIntoCubePos) { //driving into the cube with intake/tunnel on
          mRobotDrive.arcadeDrive(-0.35, 0);
          mIntakeSpin.set(.35); // will turn off intake/tunnel moters 2 bits later (as of 4/2/23)
          mTunnelSpin.set(-.4);
          mGrabber.set(.35);
        } else {
          mRobotDrive.arcadeDrive(0, 0);
          autoIntoCube = true;
        }
      }
      if (!autoCubeNodeTurn && autoIntoCube && (m_autoSelected == kRed2Auto)) {
        if (gryoCubeNodeAngleRED >= mCurrentAngle) { //turns 35 degrees CW (we want 40 from the overshoot)
          mRobotDrive.arcadeDrive(0, -0.35); // turn makes robot face the cube node
        } else {
          mRobotDrive.arcadeDrive(0, 0);
          mLeftEncoder.setPosition(0);
          mRightEncoder.setPosition(0);
          autoCubeNodeTurn = true;
          autoMoreCubeInTime = Timer.getFPGATimestamp();
        }
      }

      if (!autoCubeNodeTurn && autoIntoCube && (m_autoSelected == kBlue2Auto)) {
        if (gryoCubeNodeAngleBLUE <= mCurrentAngle) { //turns 35 degrees CCW (we want 40 from the overshoot)
          mRobotDrive.arcadeDrive(0, 0.35); // turn makes robot face the cube node
        } else {
          mRobotDrive.arcadeDrive(0, 0);
          mLeftEncoder.setPosition(0);
          mRightEncoder.setPosition(0);
          autoCubeNodeTurn = true;
          autoMoreCubeInTime = Timer.getFPGATimestamp();
        }
      }

      if (!autoAtCubeNode && autoCubeNodeTurn) {
        mRobotDrive.arcadeDrive(0, 0);
        if (autonCurrentTime - autoMoreCubeInTime >= 3) {
          mIntakeSpin.set(0); //stops tunnel/intake motors after extra intaking time
          mTunnelSpin.set(0);
          mGrabber.set(0);
        }
      }

      /*
      if (!autoAtCubeNode && autoCubeNodeTurn) { //driving to cube node --> pulls in intake, then arm out
        if (mLeftEncoder.getPosition() < autoCubeNodePos) { // 194"
          mRobotDrive.arcadeDrive(0.48, 0);
        } else {
          autonGPreleaseTime2 = Timer.getFPGATimestamp(); //next bit is time based and needs this for its start time
          mRobotDrive.arcadeDrive(0, 0);
          autoAtCubeNode = true; //condition to breakout of large "if"
        }
        if (!autoIntakeIn && autoCubeNodeTurn) {
          if (mIntakeExtEncoder.getPosition() > closedIntake) {
            mIntakeExt.set(-0.2);
          } else {
            mIntakeExt.set(0);
            autoMoreCubeInTime = Timer.getFPGATimestamp(); //start time for extra intaking (next bit)
            autoIntakeIn = true; //breakout of small "if" into arm out
          }
        }
        if (!AutocubeInMore && autoIntakeIn){
          if((autonCurrentTime - autoMoreCubeInTime <= 0)){

          } else{
            mArmEncoder.setPosition(0);
            mIntakeSpin.set(0); //stops tunnel/intake motors after extra intaking time
            mTunnelSpin.set(0);
            mGrabber.set(0);
            autonArmOutTime = Timer.getFPGATimestamp(); //start time for arm out (next bit)
            AutocubeInMore = true;
          }
        }
        if (!autoArmExtend2 && AutocubeInMore) { //arm out
          if ((mArmEncoder.getPosition() > maxArm) && (autonCurrentTime - autonArmOutTime <= 2)) {
            mArm.set(speedOut);
          }
          else {
            mArm.stopMotor();
            autoArmExtend2 = true;
          }
          }
      }
      
      if (!autoGPrelease2 && autoAtCubeNode) { //shoots out cube
        mGrabber.set(-0.35);
        if ((autonCurrentTime - autonGPreleaseTime2) >= 0.75){
          mGrabber.stopMotor();
          autoGPrelease2 = true;
        }
      }
    */
    }

  } else {
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
      mIntakeExt.stopMotor();
      mIntakeSpin.stopMotor();
      mTunnelSpin.stopMotor();
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
    if ((mStick.getRawButton(12))){
      mRightEncoder.setPosition(0);
      mLeftEncoder.setPosition(0);
      mArmEncoder.setPosition(0);
      mIntakeExtEncoder.setPosition(0);
      mGyro.reset();
    }

/*
    if ((mStick.getRawButton(1)) || (mStick.getRawButton(3))) {
      intakeStartTime = Timer.getFPGATimestamp();
      intaking = true;
    }
    
    if (intaking) {
      if((Timer.getFPGATimestamp() - intakeStartTime) < 5){
        mIntakeSpin.set(.25);
        mTunnelSpin.set(-.25);
        mGrabber.set(.35);
      } else {
        mIntakeSpin.stopMotor();
        mTunnelSpin.stopMotor();
        mGrabber.stopMotor();
        intaking = false;
      }
    }
*/

    //Grabber control
    if (mXbox.getLeftBumper()) { // Push cube out of robot (down the tunnel)
      mGrabber.set(-0.35);
      mIntakeSpin.set(-.25);
      mTunnelSpin.set(.25);
    }
    else if (mXbox.getLeftTriggerAxis() == 1){ //Pickup CONE
      mGrabber.set(-0.45);
    } 
    else if ((mStick.getRawButton(3))){
      mGrabber.set(-0.15);
    }
    else if (mXbox.getRightBumper()) { // Pull cube into robot (up the tunnel)
      mGrabber.set(0.35);
      mIntakeSpin.set(.35);
      mTunnelSpin.set(-.4);
    }
    else if (mXbox.getRightTriggerAxis() == 1){ //Pickup CUBE
      mGrabber.set(0.35);
    }    
    else {
      mGrabber.stopMotor();
      mIntakeSpin.stopMotor();
      mTunnelSpin.stopMotor();
    }

    /*
    Arm Control
      Y button = arm out (hold)
      A button = arm in (hold)
    */
    if (mXbox.getYButton()) { // Move arm out
      mArm.set(speedOut);
    }
    else if (mXbox.getAButton()) { // Move arm in
      mArm.set(speedIn);
    }
    else if ((mXbox.getPOV()==90) || (mXbox.getPOV()==270 || mArmGoToMID)){
      //extend arm to MID
      mArmGoToMID = true;
      if (mArmEncoder.getPosition() > midArm+2){
        mArm.set(speedOut);
      }
      else if (mArmEncoder.getPosition() < midArm-2) {
        mArm.set(speedIn);
      }
      else {
        mArm.stopMotor();
      }
      if (mArmEncoder.getPosition() >= (midArm-5) && mArmEncoder.getPosition() <= (midArm+5)) {
        mArmGoToMID = false;
      }
      }
  
    else {
      mArm.stopMotor();
      mArmGoToMID = false;
      mArmGoToMAX = false;
      mArmGoToHOME = false;
    }

    //Intake logic
    /*
    Intake control 
      Intake In = DPad UP
      Intake Out = DPad DOWN  (mXbox.getPOV() == 180)
    */

    if (mXbox.getXButton() || intakeExtend){
      intakeExtend = true;
      if (mIntakeExtEncoder.getPosition() < intakeOut) {
        mIntakeExt.set(.2);
      }
      else {
        mIntakeExt.stopMotor();
        intakeExtend = false;
      }
    }
    else if (mXbox.getBButton() || intakeIn){
      intakeIn = true;
      if (mIntakeExtEncoder.getPosition() >= closedIntake) {
        mIntakeExt.set(-.2);
      }
      else {
        mIntakeExt.stopMotor();
        intakeIn = false;
      }
    }
    else if (mXbox.getPOV()==0) {  //Move Intake IN
        mIntakeExt.set(-0.1);
        }
    else if (mXbox.getPOV()==180){  // Move Intake OUT
        mIntakeExt.set(0.1);
        }
    else {
      mIntakeExt.stopMotor();
      intakeExtend = false;
      intakeIn = false;
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
    autonIntakeExtend = false;
    autoCubeTurn = false;
    autoIntoCube = false;
    autoCubeNodeTurn = false;
    encoderReset = false;
    autoNotTipped = false;
    autoAtCubeNode = false;
    autoGPrelease2 = false;
    autoIntakeIn = false;
    intakeExtend = false;
    intakeIn = false;
    autoArmExtend2 = false;
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
    autonIntakeExtend = false;
    autoCubeTurn = false;
    autoIntoCube = false;
    autoCubeNodeTurn = false;
    autoAtCubeNode = false;
    autoArmExtend2 = false;
    encoderReset = false;
    autoGPrelease2 = false;
    autoIntakeIn = false;
    intakeExtend = false;
    intakeIn = false;
    autoNotTipped = false;
    AutocubeInMore = false;
  }
}