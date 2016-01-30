#include "WPILib.h"

#define JOYSTICK_DEADBAND (0.1)
//#define M_AUTO_STOP (0)
//#define M_AUTO_DRIVE_ONE (1)
//#define M_AUTO_LIFT (2)
#define M_AUTO_DRIVE_TWO (3)
#define M_AUTO_DROP (4)
#define M_MONTANA (0) 	  // ON STATE
#define M_CALIFORNIA (1) // OFF STATE
#define M_NOAH (2)	// NOTHING TO US

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
class Robot: public IterativeRobot {
	// Channels for the wheels
	RobotDrive robotDrive; // robot drive system
	Joystick m_RichardRStick;			//Richard right
	Joystick m_AaronLStick;			//Aaron left
	//LiveWindow *lw;
	//int autoLoopCounter;
	Timer m_timer;
	//RobotDrive robotDrive;	// robot drive system
	int m_autoCase = M_CALIFORNIA;
	int m_counter =0;
	const static int LeftChannel2 = 3;		//LeftChannel2 motor
	const static int LeftChannel1 = 2;		//LeftChannel1 motor
	const static int RightChannel2 = 1;		//RightChannel2 motor
	const static int RightChannel1 = 0;		//RightChannel1 motor
	AnalogInput m_sonicAlex;			//ultra sonic sensor when we use this later put m_sonicAlex.Get()    Operates from 2.5-5.5V

	//const static int joystickChannel = 0;

	//bool m_driving;
	//bool m_disabled;

	//RobotDrive robotDrive;	// robot drive system
	//Joystick rightStick;			// right joystick
	//Joystick m_leftStick;		// left Joystick
	//Talon m_leftArmMotor;			// left arm motor
	//Talon m_rightArmMotor;      // right arm motor
	//Talon m_plateElevatorMotor; // plate elevator motor
	//DigitalInput m_topLimitSwitch;
	//DigitalInput m_bottomLimitSwitch;
	//Timer m_timer;
	//double m_autoDriveTimer;
	//int m_autoCase = M_AUTO_STOP; 	//Autonomous state machine

	// Solenoids to control with the joystick.
	// Solenoid corresponds to a single solenoid.
	//	Solenoid m_solenoid;
	// DoubleSolenoid corresponds to a double solenoid.
	//DoubleSolenoid m_armDeployer;
	//DoubleSolenoid m_plateTilt;
	//Solenoid m_LED;

	// Numbers of the buttons to use for triggering the solenoids.
	//const int kSolenoidButton = 1;
	//const int kDoubleSolenoidForward = 2;
	//const int kDoubleSolenoidReverse = 3;

public:
	Robot() :
		robotDrive(LeftChannel2, LeftChannel1, RightChannel2, RightChannel1), // these must be initialized in the same order
		m_RichardRStick(0), // initialize the joystick on port 0 (right side)
		m_AaronLStick(1), // initialize the Joystick on port 1 (left side)
		m_sonicAlex(0)

//rightStick(joystickChannel),		// as they are declared above.
//m_leftStick(1), // Initialize Joystick on port 1
//m_leftArmMotor(4), // Initialize the Talon on channel 4.
//m_rightArmMotor(5),
//m_plateElevatorMotor(6),
//m_topLimitSwitch(0),
//m_bottomLimitSwitch(1),
//	m_solenoid(0), // Use single solenoid on channel 0.
//m_armDeployer(1, 2), // Use double solenoid with Forward Channel of 1 and Reverse of 2.
//m_plateTilt(3, 4) // Use double solenoid with Forward Channel of 3 and Reverse of 4.
//m_LED(7) //LED solenoid lights
//	m_elevatorTilt(5, 6)    // Use double solenoid with Forward Channel of 5 and Reverse of 6.
{
		robotDrive.SetExpiration(0.1);	//Number of seconds it takes to timeout
		robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);	// invert the left side motors
		robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);// you may need to change or remove this to match your robot
}

	/**
	 * Runs the motors with Mecanum drive.
	 */
	void AutonomousInit() {
		m_timer.Reset();
		m_timer.Start();
		//	m_autoDriveTimer = SmartDashboard::GetNumber("Autonomous Drive Timer", 4.0);
		//	SmartDashboard::PutNumber("Autonomous Drive Timer", m_autoDriveTimer);
		m_autoCase =  M_MONTANA;
	}

	/*void DisabledInit(){
	 m_disabled=true;
	 }

	 void DisabledPeriodic(){
	 m_disabled=true;
	 SmartDashboard::PutBoolean("Disabled",m_disabled);
	 }*/

	void AutonomousPeriodic() {
		if (m_counter >= 4) {				//does the counter equal 4?
			m_autoCase = M_NOAH;
		}
		if (m_autoCase ==M_MONTANA) {	  // does it equal MONTANA?
			if (m_timer.Get() <= 3.0) {  //the robot is on for 3 seconds
				robotDrive.TankDrive(0.266, 0.266);			//first number is the smaller motor, the second number is the bigger motor
			}
			else {
				m_autoCase = M_CALIFORNIA;
			}
		}
		if (m_autoCase == M_CALIFORNIA) {						//does it equal CALIFORNIA?
			if (m_timer.Get() > 3.0 && m_timer.Get() <= 4.5) { // the robot will be off for 1.5 seconds
				robotDrive.TankDrive(0.0, 0.0);
			}
			else {
				m_timer.Reset ();
				m_autoCase = M_MONTANA;
				m_counter = m_counter +1;
			}
		}
		if (m_autoCase == M_NOAH) {
			robotDrive.TankDrive(0.0, 0.0);
			// NO QUIT GO HOME TRATORS NOT UP FOR DBAT8
		}
	}
	void TeleopInit() {

	}

	void TeleopPeriodic() {
		SmartDashboard::PutNumber("m_sonicAlex",m_sonicAlex.GetVoltage());
		//system.out.Println(m_sonicAlex.GetVoltage());
		if (m_sonicAlex.GetVoltage() <3) {
			robotDrive.TankDrive(m_RichardRStick, m_AaronLStick);

		}

			/*		SmartDashboard::PutNumber("Left Stick Y", m_leftStick.GetY());
		SmartDashboard::PutBoolean("Top Limit Switch", m_topLimitSwitch.Get());
		SmartDashboard::PutBoolean("Bottom Limit Switch", m_bottomLimitSwitch.Get());
		SmartDashboard::PutNumber("Z Axis", rightStick.GetZ());
		SmartDashboard::PutNumber("Y Axis", rightStick.GetY());*/


	}
};

START_ROBOT_CLASS(Robot)
