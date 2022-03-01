package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ConstantsPorts;
import frc.robot.Constants.ConstantsValues;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;


public class IntakeSubsystem extends SubsystemBase {

    CANSparkMax intake;
    RelativeEncoder intakeEncoder;
    Solenoid lifter;
    NetworkTableInstance ntInst;
    boolean previousLifterState;

    boolean on = false;

    public IntakeSubsystem() {
        intake = new CANSparkMax(ConstantsPorts.intakeID, MotorType.kBrushless);
        intake.setInverted(true);

        intakeEncoder = intake.getEncoder();

        lifter = new Solenoid(PneumaticsModuleType.REVPH, ConstantsPorts.lifterID);

        if (Robot.isSimulation()) {
        }

        ntInst = NetworkTableInstance.getDefault();
        ntInst.getTable("CommandoDash").getSubTable("SensorData")
            .getEntry("intakeSolenoidState").setBoolean(lifter.get());
    }
    

    //running

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

    //lifting/solenoid stuff

    //raise lifter
    public void extend() {
        boolean on = true;
        lifter.set(on);
    }

    //lower lifter
    public void retract() {
        boolean on = false;
        lifter.set(on);
    }

    //toggle
    public void toggleExtend() {    
        on = !on;
        lifter.toggle();
    }

    //sensors

    //get velocity
    public void getVelocity() {
        intakeEncoder.getVelocity();
    }

    //reset encoder
    public void resetEncoder() {
        intakeEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        //Update CDD with the solenoid state
        boolean currLifterState = on;
        if (previousLifterState != currLifterState) {
            ntInst.getTable("CommandoDash").getSubTable("SensorData")
                .getEntry("intakeSolenoidState").setBoolean(currLifterState);
        }
        previousLifterState = currLifterState;
    }
}
