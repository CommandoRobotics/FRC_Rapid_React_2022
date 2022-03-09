package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsField;
import frc.robot.Constants.ConstantsPorts;
import frc.robot.Constants.ConstantsValues;


public class IntakeSubsystem extends SubsystemBase {

    //Motors
    CANSparkMax intake;

    //Solenoids
    DoubleSolenoid lifter;
    NetworkTableInstance ntInst;
    boolean previousLifterState;

    //Sensors
    RelativeEncoder intakeEncoder;
    PhotonCamera CargoHound;

    //Constants
    boolean on = false;
    Alliance previousAlliance = Alliance.Blue;
    DriveSubsystem driveSubsystem;

    //Odometry
    Field2d field;

    public IntakeSubsystem(DriveSubsystem driveSubsystem) {
        intake = new CANSparkMax(ConstantsPorts.intakeID, MotorType.kBrushless);
        intake.setInverted(true);

        intakeEncoder = intake.getEncoder();
        // intakeEncoder.setVelocityConversionFactor(0);
        CargoHound = new PhotonCamera("CargoHound");
        field = (Field2d) SmartDashboard.getData("Field");

        lifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, ConstantsPorts.lifterForwardId, ConstantsPorts.lifterReverseId);

        ntInst = NetworkTableInstance.getDefault();
        ntInst.getTable("CommandoDash").getSubTable("SensorData")
            .getEntry("intakeSolenoidState").setBoolean(lifter.isFwdSolenoidDisabled());
        
        this.driveSubsystem = driveSubsystem;
    }
    

    //Motors

    //set power
    public void setPower(double power) {
        intake.set(power);
    }

    //set volatage
    public void setVoltage(double Volts) {
        intake.setVoltage(Volts);
        
    }

    //stop
    public void stop() {
        intake.stopMotor();
    }

    //intakeIn()
    public void intakeIn() {
        intake.set(ConstantsValues.intakePower);
    }

    //intakeOut()
    public void intakeOut() {
        intake.set(ConstantsValues.ejectPower);
    }

    //Solenoids

    //raise lifter
    public void extend() {
        on = true;
        lifter.set(Value.kReverse);
    }

    //lower lifter
    public void retract() {
        on = false;
        lifter.set(Value.kForward);
    }

    //toggle
    public void toggleExtend() {    
        if(on) {
            retract();
        } else {
            extend();
        }
    }

    //Sensors

    //get velocity
    public void getVelocity() {
        intakeEncoder.getVelocity();
    }

    //reset encoder
    public void resetEncoder() {
        intakeEncoder.setPosition(0);
    }

    //CargoHound

    /**
     * Retrieves the latest data from the CargoHound PhotonVisionCamera
     * @return the lastest result as a PhotonPipelineResult
     */
    public PhotonPipelineResult getHoundData() {
        return CargoHound.getLatestResult();
    }

    /**
     * Toggles "DriverMode" of the CargoHound. Driver Mode disables vision
     * processing and changes the camera settings and stream resolution for 
     * using CargoHound as a driver camera.
     */
    public void toggleDriverModeCargoHound() {
        CargoHound.setDriverMode(!CargoHound.getDriverMode());
    }

    /**
     * Sets the pipeline of the CargoHound. Currently, 0 is Blue and 1 is Red
     * @param pipeline index of the desired pipeline
     */
    public void setHoundPipeline(int pipeline) {
        CargoHound.setPipelineIndex(pipeline);
    }

    /**
     * Uses PhotonUtil and known constants to find the distance to a given target. 
     * Gets data about the target through a PhotonPipelineResult.<p>
     * </p>Returns -1 if no targets found.
     * @param result
     * @return the distance to the target or -1 if there are no targets
     */
    public double getDistanceToCargo(PhotonPipelineResult result) {
        if (result.hasTargets()) {
            double distance = PhotonUtils.calculateDistanceToTargetMeters(
                ConstantsField.CAMERA_HEIGHT_METERS,
                ConstantsField.TARGET_HEIGHT_METERS,
                ConstantsField.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
            SmartDashboard.putNumber("HoundDis", distance);
            return distance;
        } else {
            SmartDashboard.putNumber("HoundDis", -1);
            return -1;
        }
    }

    /**
     * Takes an input and determines if the input is less than the minimum output. 
     * This is applied to both + and - numbers. For example, a min of 0.1 will scale everything
     * below 0.1 and above -0.1 to 0.1 and -0.1 respectively
     * @param input
     * @param minOutput
     */
    public static double scaleAroundZero(double input, double minOutput) {
        if (input < 0) {
            if (input >= -minOutput) {
                return -minOutput;
            } else {
                return input;
            }
        } else if (input > 0) {
            if (input <= minOutput) {
               return minOutput;
            } else {
                return input;
            }
        } else {
            return 0;
        }
    }

    /**
     * Returns the position of the cargo as a Translation2d relative to the robot. It used a PhotonPipelineResult
     * to determine the offsets of the robot by getting distance and yaw from the PipelineResult
     * @param result PhotonPipelineResult with the tracked cargo
     * @return
     */
    public Translation2d estimateRobotToCargoTransformation(PhotonPipelineResult result) {
        if (result.hasTargets()) {
            //TODO Verify Yaw needs to be reversed
            return PhotonUtils.estimateCameraToTargetTranslation(getDistanceToCargo(result), Rotation2d.fromDegrees(-result.getBestTarget().getYaw()))
                .plus(ConstantsValues.houndToRobotTranslation2d);
        } else {
            System.out.println("estimateRobotToCargoTransformation() was not able to find a target and therefore returned (0,0)");
            return new Translation2d(0,0);
        }
    }

    public Pose2d estimateCargoFieldPose2d(PhotonPipelineResult result, Pose2d robotPose) {
        return robotPose.transformBy(
            new Transform2d(
                estimateRobotToCargoTransformation(result),
                Rotation2d.fromDegrees(0)));
    }


    @Override
    public void periodic() {
        //Display Pose2d of the ball to the dash
        if (getHoundData().hasTargets()) {
            field.getObject("SeenCargo").setPose(
                estimateCargoFieldPose2d(
                    getHoundData(), 
                    driveSubsystem.getPose()));
        } 

        //Update the pipeline of the CargoHound. Default is Blue
        if (previousAlliance != DriverStation.getAlliance()) {
            if (DriverStation.getAlliance() == Alliance.Red) {
                setHoundPipeline(1);
            } else {
                setHoundPipeline(0);
            }
            previousAlliance = DriverStation.getAlliance();
        }

        //Update CDD with the solenoid state
        boolean currLifterState = on;
        if (previousLifterState != currLifterState) {
            ntInst.getTable("CommandoDash").getSubTable("SensorData")
                .getEntry("intakeSolenoidState").setBoolean(currLifterState);
        }
        previousLifterState = currLifterState;
    }
}
