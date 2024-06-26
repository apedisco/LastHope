package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private boolean isSlowMode;
    public boolean isLocked;
    public Pigeon2 mPigeon2;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

//     AutoBuilder.configureHolonomic(
//         this::getPose, // Robot pose supplier
//         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
//         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//         this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
//         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
//                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
//                 new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
//                 4.5, // Max module speed, in m/s
//                 0.4, // Drive base radius in meters. Distance from robot center to furthest module.
//                 new ReplanningConfig() // Default path replanning config. See the API for the options here
//         ),
//         () -> {
//           // Boolean supplier that controls when the path will be mirrored for the red alliance
//           // This will flip the path being followed to the red side of the field.
//           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

//           var alliance = DriverStation.getAlliance();
//           if (alliance.isPresent()) {
//             return alliance.get() == DriverStation.Alliance.Red;
//           }
//           return false;
//         },
//         this // Reference to this subsystem to set requirements
// );

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        mPigeon2 = new Pigeon2(0);
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        mPigeon2 = new Pigeon2(0);
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(0, 0, 0),
                                            new PIDConstants(0, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this); // Subsystem for requirements
    }
    // public void getPose(){
    //     return Odometry.getposeMeters();
    // }
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
    public void drive(ChassisSpeeds chassisSpeeds){
        isSlowMode = !isSlowMode;
    }
    public void rotation(double rotationRate, SwerveDrivetrain drivetrain, SwerveRequest.FieldCentric driveRequest){
        drivetrain.setControl( driveRequest.withVelocityX(0 ) // Drive forward with
            // negative Y (forward)
            .withVelocityY(0 ) // Drive left with negative X (left)
            .withRotationalRate(rotationRate) // Drive counterclockwise with negative X (left)
        );
    }

    public void moveLinear(double xVel, double yVel, SwerveDrivetrain drivetrain, SwerveRequest.FieldCentric driveRequest){
        System.out.println("working");
        drivetrain.setControl( driveRequest.withVelocityX(xVel ) // Drive forward with
            // negative Y (forward)
            .withVelocityY(yVel ) // Drive left with negative X (left)
            .withRotationalRate(0) // Drive counterclockwise with negative X (left)
        );
    }

    public void ResetFieldOrientation(){
        System.out.println("resetting");
        mPigeon2.reset();
      }
      public void CheckMotorVoltage(SwerveModuleConstants... modules){
        SmartDashboard.putNumber("FrontLeftDrive", getModule(0).getDriveMotor().getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("FrontLeftSteer", getModule(0).getSteerMotor().getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("FrontRightDrive", getModule(1).getDriveMotor().getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("FrontRightSteer", getModule(1).getSteerMotor().getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("BackLeftDrive", getModule(2).getDriveMotor().getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("BackLeftSteer", getModule(2).getSteerMotor().getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("BackRightDrive", getModule(3).getDriveMotor().getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("BackRightSteer", getModule(3).getSteerMotor().getSupplyVoltage().getValueAsDouble());
      }
     
    //   public void ZeroYawAndGyroZ(){
    //     mPigeon2.
    //   }

}
