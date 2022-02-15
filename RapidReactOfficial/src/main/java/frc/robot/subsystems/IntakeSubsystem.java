package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsPorts;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;


public class IntakeSubsystem extends SubsystemBase {

    CANSparkMax intake;
    RelativeEncoder intakeEncoder;
    Solenoid lifter;

    boolean on = true;

    public IntakeSubsystem() {
        intake = new CANSparkMax(ConstantsPorts.intakeID, MotorType.kBrushless);

        intakeEncoder = intake.getEncoder();

        lifter = new Solenoid(PneumaticsModuleType.REVPH, ConstantsPorts.lifterID);
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
        intake.set(ConstantsPorts.IntakePower);
    }

    //intakeOut()
    public void intakeOut() {
        intake.set(ConstantsPorts.EjectPower);
    }

    //lifting/solenoid stuff

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

    //sensors

    //get velocity
    public void getVelocity() {
        intakeEncoder.getVelocity();
    }

    //reset encoder
    public void resetEncoder() {
        intakeEncoder.setPosition(0);
    }
}
