# Stronghold_2016
backup code


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
//the scissors lift will be named Lizard Lift in the code
//the game pad controller will be named Ben in the code


/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
class Robot: public IterativeRobot {
	// Channels for the wheels
	RobotDrive robotDrive; // robot drive system
	Joystick m_RichardRightStick;			//Richard= right joystick
	Joystick m_AaronLeftStick;			//Aaron= left joystick
	Talon m_lizardLiftMotor;					//the scissor lift motor
	DigitalInput m_topLimitSwitch;
	DigitalInput m_bottomLimitSwitch;

	//LiveWindow *lw;
	//int autoLoopCounter;
	Timer m_timer;
	AnalogInput m_sonicAlex;			//ultra sonic sensor when we use this later put m_sonicAlex.Get()    Operates from 2.5-5.5V
	//RobotDrive robotDrive;	// robot drive system
	int m_autoCase = M_CALIFORNIA;
	int m_counter =0;
	const static int LeftChannel2 = 3;		//LeftChannel2 motor
	const static int LeftChannel1 = 2;		//LeftChannel1 motor
	const static int RightChannel2 = 1;		//RightChannel2 motor
	const static int RightChannel1 = 0;		//RightChannel1 motor

	//const static int joystickChannel = 0;

	//bool m_driving;
	//bool m_disabled;

	//RobotDrive robotDrive;	// robot drive system
	//Joystick rightStick;			// right joystick
	//Joystick m_leftStick;		// left Joystick
	//Talon m_leftArmMotor;			// left arm motor
	//Talon m_rightArmMotor;      // right arm motor
	//Talon m_lizardLiftMotor; // plate elevator motor
	//DigitalInput m_topLimitSwitch;
	//DigitalInput m_bottomLimitSwitch;
	//Timer m_timer;
	//double m_autoDriveTimer;
	//int m_autoCase = M_AUTO_STOP; 	//Autonomous state machine



	Encoder *enc;
	double m_tempvalue = 0.0;

	CameraServer *cam;

public:
	Robot() :
		robotDrive(LeftChannel2, LeftChannel1, RightChannel2, RightChannel1), // these must be initialized in the same order
		m_RichardRightStick(0), // initialize the joystick on port 0 (right side)
		m_AaronLeftStick(1), // initialize the Joystick on port 1 (left side)
		m_sonicAlex(0)

//rightStick(joystickChannel),		// as they are declared above.
//m_leftStick(1), // Initialize Joystick on port 1
//m_leftArmMotor(4), // Initialize the Talon on channel 4.
//m_rightArmMotor(5),
//m_lizardLiftMotor(6),
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

		enc = new Encoder(4, 5, false, Encoder::EncodingType::k4X);	//first two equal port numbers
		// enc->SetMaxPeriod(0.1);  (Not needed)
		enc->SetMinRate(10);
		enc->SetDistancePerPulse(5);
		enc->SetReverseDirection(true);
		enc->SetSamplesToAverage(7);

		cam= CameraServer::GetInstance();
		cam->SetQuality(50);
		cam->StartAutomaticCapture("cam1");
					//   http://roborio-4243-frc.local/ must use Internet Explorer to find where
			//this is located at while connected to the robo rio
			//the camera name (ex "cam0") can be found through the roborio web interface
		//CameraServer::GetInstance()->StartAutomaticCapture("cam1");
		//takes around 40 seconds to come up.
		// sometimes need to delete the usb webcam box and put a new one in place of it
		// go chance that we will have to lower the resolution for competition

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

		SmartDashboard::PutNumber("distance per pulse 1 ", m_tempvalue);
			//this has to be here in order to get PutNumber to work in teleop periodic
	}

	void TeleopPeriodic() {


		SmartDashboard::PutNumber("encoder count ", enc->Get());
		m_tempvalue = SmartDashboard::GetNumber("distance per pulse 1 ", -1.0);
			//default is negative to make it easy to see
			//this has to be the same label in teleop init
		//SmartDashboard::PutNumber("distance per pulse ", m_tempvalue);
		enc->SetDistancePerPulse(m_tempvalue);
		SmartDashboard::PutNumber("encoder direction ", enc->GetDirection());
		SmartDashboard::PutNumber("encoder distance ", enc->GetDistance());

		SmartDashboard::PutNumber("ultraSonicSensor ",m_sonicAlex.GetVoltage());
		//   http://www.maxbotix.com/articles/032.htm
		//how to translate voltage to distance.

		//SmartDashboard::PutNumber("m_sonicAlex",4.0);
		//system.out.Println(m_sonicAlex.GetVoltage());
		//robotDrive.TankDrive(m_RichardRightStick, m_AaronLeftStick);
		if (m_sonicAlex.GetVoltage() > 0.3) {
			robotDrive.TankDrive(m_RichardRightStick, m_AaronLeftStick);
		}
		else {
			robotDrive. TankDrive(0.0, 0.0);
		}
		//this is for the Lizard Lift
		if (m_AaronLeftStick.GetY() < -JOYSTICK_DEADBAND) { //need to declaring vars.
					//If joystick is pushed away from the operator along the Y axis
					//Deadband (0.1) used to prevent unwanted movement
					if (m_topLimitSwitch.Get()) {
						//If the top limit switch is not depressed
						//Switch normally open
						if (m_AaronLeftStick.GetRawButton(2)) {		//If button 2 is pressed
							// Set the motor controller's output.
							// This takes a number from -1 (100% speed in reverse) to +1 (100% speed forwards).
							m_lizardLiftMotor.SetSpeed(m_AaronLeftStick.GetY());//Run plate motor at full power
						} else {		//If not pressed
							m_lizardLiftMotor.SetSpeed(0.75 * (m_AaronLeftStick.GetY())); //0.75 gain used to control motor speed
						}
					} else {		//If top limit switch is depressed
						m_lizardLiftMotor.SetSpeed(0.0);		//Stop plate motor
					}
				} else if (m_AaronLeftStick.GetY() > JOYSTICK_DEADBAND) {
					//If joystick is pulled towards operator along the Y axis
					//Deadband (0.1) used to prevent unwanted movement
					if (m_bottomLimitSwitch.Get()) {
						//If bottom limit switch is not depressed
						//Switch normally open
						m_lizardLiftMotor.SetSpeed(0.5 * (m_AaronLeftStick.GetY()));
					} else {
						m_lizardLiftMotor.SetSpeed(0.0);
					}
				}

	}
};

START_ROBOT_CLASS(Robot)
