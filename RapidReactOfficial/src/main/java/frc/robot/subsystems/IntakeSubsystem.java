package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ConstantsField;
import frc.robot.Constants.ConstantsPorts;
import frc.robot.Constants.ConstantsValues;
import frc.robot.commands.HoundCargo;

import javax.lang.model.element.ModuleElement.DirectiveKind;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSubsystem extends SubsystemBase {

    //Motors
    CANSparkMax intake;

    //Solenoids
    Solenoid lifter;

    //Sensors
    RelativeEncoder intakeEncoder;
    PhotonCamera CargoHound;

    //Constants
    boolean on = true;
    Alliance previousAlliance = Alliance.Blue;

    public IntakeSubsystem() {
        intake = new CANSparkMax(ConstantsPorts.intakeID, MotorType.kBrushless);
        intake.restoreFactoryDefaults();
        intake.setInverted(false);

        intakeEncoder = intake.getEncoder();
        // intakeEncoder.setVelocityConversionFactor(0);
        CargoHound = new PhotonCamera("CargoHound");

        lifter = new Solenoid(PneumaticsModuleType.REVPH, ConstantsPorts.lifterID);
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
    public void raiseLifter() {
        boolean on = true;
        lifter.set(on);
    }

    //lower lifter
    public void lowerLifter() {
        boolean on = false;
        lifter.set(on);
    }

    //toggle
    public void toggleLifter() {       
        on = !on;
        lifter.toggle();
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

    public PhotonPipelineResult getHoundData() {
        return CargoHound.getLatestResult();
    }

    public void toggleDriverModeCargoHound() {
        CargoHound.setDriverMode(!CargoHound.getDriverMode());
    }

    public void setHoundPipeline(int pipeline) {
        CargoHound.setPipelineIndex(pipeline);
    }

    /**
     * Returns -1 if no targets found
     * @param result
     * @return
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
     * Takes an input and determines if that inputs is less than the minimum output. 
     * This is applied to both + and - numbers. For example, a min of 0.1 will scale everything
     * below 0.1 and above -0.1 to 0.1 and -0.1 respectively
     * @param input
     * @param minOutput
     * @return
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


    @Override
    public void periodic() {
        //Display pose2d of the ball to the dash
        
        //Update the pipeline of the CargoHound
        if (previousAlliance != DriverStation.getAlliance()) {
            if (DriverStation.getAlliance() == Alliance.Red) {
                setHoundPipeline(1);
            } else {
                setHoundPipeline(0);
            }
            previousAlliance = DriverStation.getAlliance();
        }
    }
}
