// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the 
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private static final int kJoystickPort = 0;
  // private static final int kHoodJoystickPort = 1; // this should be for the second joystick if it doesnt work keep on changing
  private static final boolean testingSparkMAX = false;
  private static final int motorPort = 20;
  private static final int motorPort2 = 21;
  private static final int hoodMotorPort = 22; // EDIT THIS ONCE U FIND MOTOR PORT FOR SPARK MAX MC
  private static final int indexerPort = 6; // spark max
  private static final int funnelPort = 7; // spark max
  private static final int rollerPort = 8; // spark max
  private static final int armPort = 5; // spark max
  private static final double maxSpeed = 1.000; // this is for the  ball                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             ;
  private static final boolean inverted = true;
  private static final boolean hoodInverted = true;
  private static final boolean indexerInverted = true; // modify
  private static final boolean funnelInverted = true; // modify
  private static final boolean rollerInverted = true; // modify
  private static final boolean armInverted = true; //modify

  private static final int XBOX_LEFT_X_AXIS = 0;
  private static final int XBOX_LEFT_Y_AXIS = 1;
  private static final int XBOX_RIGHT_X_AXIS = 4;
  private static final int XBOX_RIGHT_Y_AXIS = 5;
  private static final double hoodSpeed = 0.05; // CHANGE SPEED IF TOO SLOW FOR HOOD

  private static final double indexerSpeed = 0.9; // modify
  private static final double funnelSpeed = 0.35; // modify
  private static final double rollerSpeed = 0.8; //modify
  private static final double armSpeed = 0.1;
  private static final double kP = 0.01;

  private XboxController joystick = new XboxController(kJoystickPort); // Joystick
  // private XboxController hood_joystick = new XboxController(kHoodJoystickPort);
  private CANSparkMax sm_motor;
  private WPI_TalonFX tfx_motor;
  private WPI_TalonFX tfx2_motor;
  private CANSparkMax hood_motor;
  private CANSparkMax indexer_motor;
  private CANSparkMax funnel_motor;
  private CANSparkMax roller_motor;
  private CANSparkMax arm_motor;
  private static int hoodMode = 0;

  private static final int gearTeeth = 33;
  private static final int hoodTeeth = 20;

  //public static double kIndexerSpeed = 0.1;


  private static final boolean smart = false;

  private static final boolean runIndex = true;

  private static final boolean runFunnel = true;

  private static final boolean runRollers = true;

  private static final boolean runArm = true;

  private static final boolean runDT = true;

  private static final boolean runShooter = true;

  private static final boolean runHood = true;

  private static boolean indexRun = false;
  private static boolean funnelRun = false;
  private static int rollerRun = 1;
  private static boolean shooterRun = false;
  private static int hoodRun = 0;
  private static int armRun = 0;

  private RelativeEncoder m_encoder;
  //DT Constants
  public static final int kLeftMasterPort = 1;//2
  public static final int kLeftFollowerPort = 2;//1
  public static final int kRightMasterPort = 3;//4
  public static final int kRightFollowerPort = 4;//3

  public static final TalonFXInvertType kLeftInvertType = TalonFXInvertType.CounterClockwise;
  public static final TalonFXInvertType kRightInvertType = TalonFXInvertType.Clockwise;

  public static final double kMaxOutput = 0.6;
  public static double kDistancePerPulseFactor;

  //Drivetrain
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(kLeftMasterPort);
  private final WPI_TalonFX leftFollower = new WPI_TalonFX( kLeftFollowerPort);
  private final WPI_TalonFX rightMaster = new WPI_TalonFX( kRightMasterPort);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX( kRightFollowerPort);

  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(leftMaster, leftFollower);
  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(rightMaster, rightFollower);

  private final DifferentialDrive drive = new DifferentialDrive(m_leftGroup, m_rightGroup);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

  

    leftMaster.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightFollower.configFactoryDefault();

    TalonFXConfiguration configs = new TalonFXConfiguration();


    leftMaster.configAllSettings(configs);
    leftFollower.configAllSettings(configs);
    rightMaster.configAllSettings(configs);
    rightFollower.configAllSettings(configs);

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    leftMaster.setInverted( kLeftInvertType);
    leftFollower.setInverted(kLeftInvertType);
    rightMaster.setInverted( kRightInvertType);
    rightFollower.setInverted( kRightInvertType);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    if(testingSparkMAX) {
      sm_motor = new CANSparkMax(motorPort, MotorType.kBrushless);
      sm_motor.restoreFactoryDefaults();
      sm_motor.setInverted(inverted);
    }
    else {
      tfx_motor = new WPI_TalonFX(motorPort);
      tfx_motor.configFactoryDefault();
      tfx_motor.setInverted(inverted);      
      tfx2_motor = new WPI_TalonFX(motorPort2);
      tfx2_motor.configFactoryDefault();
      tfx2_motor.setInverted(inverted);
      // tfx2_motor.follow(tfx_motor);
      hood_motor = new CANSparkMax(hoodMotorPort, MotorType.kBrushless);
      hood_motor.restoreFactoryDefaults();
      hood_motor.setInverted(hoodInverted);
      
      if(runIndex) {
        indexer_motor = new CANSparkMax(indexerPort, MotorType.kBrushless);
        indexer_motor.restoreFactoryDefaults();
        indexer_motor.setInverted(indexerInverted);
      }
      if(runFunnel) {
        funnel_motor = new CANSparkMax(funnelPort, MotorType.kBrushless);
        funnel_motor.restoreFactoryDefaults();
        funnel_motor.setInverted(funnelInverted);
      }
      if (runRollers){
        roller_motor = new CANSparkMax(rollerPort, MotorType.kBrushless);
        roller_motor.restoreFactoryDefaults();
        roller_motor.setInverted(rollerInverted);
      }
      if (runArm){
        arm_motor = new CANSparkMax(armPort, MotorType.kBrushless);
        arm_motor.restoreFactoryDefaults();
        arm_motor.setInverted(armInverted);
      }
    }

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if(testingSparkMAX) {
    //   sm_motor.set(joystick.getRawAxis(XBOX_LEFT_X_AXIS) * maxSpeed);
    // }
    // else {
    //   tfx_motor.set(joystick.getRawAxis(XBOX_LEFT_X_AXIS) * maxSpeed);
    //   tfx2_motor.set(-joystick.getRawAxis(XBOX_LEFT_X_AXIS) * maxSpeed);
    //   if(smart) {
    //     if(joystick.getAButton()) {
    //       hoodMode = 0;
    //     }
    //     else if(joystick.getBButton()) {
    //       hoodMode = 1;
    //     }
    //     else if(joystick.getXButton()) {
    //       hoodMode = 2;
    //     }
    //     else if(joystick.getYButton()) {
    //       hoodMode = 3;
    //     }

    //     double goalTicks = (double)hoodMode * 3;
    //     int motorTicks = 42 * 70; // change later (google search) multiply be gear ratio


    //     // how many ticks are in the motor

    //     m_encoder = hood_motor.getEncoder();

    //     // 30 ticks in spark max - still need to clarify
    //     // program with joystick -> hood at bottom, spin to top, ahve encoder, count values
    //     // 33 ticks in grey gear
    //     // 21 ticks in the hood


    //     // goes 10 teeth to 40 teeth

    //     // 33 teeth gear
    //     // 1 rev of motor -> 33 teeth
    //     // 0 -> 0
    //     // 1 -> 7
    //     // 2 -> 14
    //     // 3 -> 21

    //     // have encoder measure how many ticks have been rotated



    //     // convert it to fraction of a rotation (position)/motorticks // done
    //     // convert the fraction to how many teeth on the hood
    //     double trn = m_encoder.getPosition()/70;
    //     double teethMove = trn * gearTeeth;
    //     // then find difference
    //     double dif = goalTicks - teethMove;
    //     // you do p * difference speed for motor
    //     double speed = kP * dif;

    //     hood_motor.set(speed);
    //     SmartDashboard.putNumber("Hood Mode", hoodMode);
    //     SmartDashboard.putNumber("Tick Goal", goalTicks);
    //     SmartDashboard.putNumber("Current Position", trn);
    //     SmartDashboard.putNumber("Difference in Ticks", dif);
    //     SmartDashboard.putNumber("Speed", speed);
    //     // System.out.println("Current Position" + trn);
    //     // System.out.println("Difference in Ticks" + dif);
    //     // System.out.println("Speed" + speed);
      // }
      // else {hood_motor.set(joystick.getRawAxis(XBOX_RIGHT_X_AXIS) * hoodSpeed);}
      if (runDT){
        drive.arcadeDrive(-joystick.getRawAxis(1), joystick.getRawAxis(4));
        drive.setMaxOutput(kMaxOutput);
      }
      if (runShooter){
      
        double shoot_speed = 0.00000;        
        if(shooterRun) shoot_speed = maxSpeed;
        tfx_motor.set(shoot_speed);
        tfx2_motor.set(-shoot_speed);
        if(joystick.getStartButtonPressed()) {
          shooterRun=!shooterRun;
          }
      }
      if (runHood){
        if(joystick.getXButtonPressed()) {
          hoodRun = 1;
        }
        else if (joystick.getXButtonReleased()){
          hoodRun = 0;
        }
        if(joystick.getAButtonPressed()) {
          hoodRun = -1;
        }
        else if (joystick.getAButtonReleased()){
          hoodRun = 0;
        }
        hood_motor.set(hoodSpeed * hoodRun);
      }   
      if(runIndex) {
        if(joystick.getRightBumper()) {
          indexRun = true;
        }
        if(joystick.getLeftBumper()) {
          indexRun = false;
        }
        double speed = 0.00000;
        if(indexRun) speed = indexerSpeed;
        indexer_motor.set(speed);
      }
      if(runFunnel) {
        if(joystick.getRightBumper()) {
          funnelRun = true;
        }
        if(joystick.getLeftBumper()) {
          funnelRun = false;
        }
        double speed = 0.0000;
        if(funnelRun) speed = funnelSpeed;
        funnel_motor.set(speed);
      }
      if (runRollers){
        if(joystick.getBackButtonPressed()) {
          if (rollerRun==1) rollerRun = 0;
          else rollerRun = 1;
          double speed = 0.0000;
          if(rollerRun==1) speed = rollerSpeed;
          roller_motor.set(speed);
      }
      if (runArm){
        if(joystick.getYButtonPressed()) {
          armRun = 1;
        }
        else if (joystick.getYButtonReleased()){
          armRun = 0;
        }
        if(joystick.getBButtonPressed()) {
          armRun = -1;
        }
        else if (joystick.getBButtonReleased()){
          armRun = 0;
        }
        arm_motor.set(armSpeed * armRun);
      }
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
