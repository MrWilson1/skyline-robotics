#include "Experimental.h"


TestMotor::TestMotor(SpeedController *motor, const char *name) :
		BaseSubsystem(name) {
	m_motor = motor;
	AddActuatorToLiveWindow("Motor", m_motor);
}

TestMotor::~TestMotor() {
	// Empty
}

void TestMotor::SetSpeed(float speed) {
	m_motor->Set(speed);
}

void TestMotor::MaxSpeed() {
	m_motor->Set(1.0);
}



TestEncoder::TestEncoder(Encoder *encoder, const char *name) :
		BaseSubsystem(name),
		m_distancePerPulse(1) {
	m_encoder = encoder;
	//m_encoder->SetDistancePerPulse(m_distancePerPulse);
	AddSensorToLiveWindow("Raw Encoder", m_encoder);
}

TestEncoder::TestEncoder(Encoder *encoder, float distancePerPulse, const char *name) :
		BaseSubsystem(name),
		m_distancePerPulse(distancePerPulse) {
	m_encoder = encoder;
	//m_encoder->SetDistancePerPulse(m_distancePerPulse);
	AddSensorToLiveWindow("Raw Encoder", m_encoder);
}

TestEncoder::~TestEncoder() {
	// Empty
}

void TestEncoder::ReportAll() {
	Log("Get", m_encoder->Get());
	Log("Get raw", m_encoder->GetRaw());
	Log("Period", m_encoder->GetPeriod());
	Log("Rate", m_encoder->GetRate());
	LogBool("Is encoder stopped?", m_encoder->GetStopped());
	Log("Distance", m_encoder->GetDistance());
	Log("PIDGet", m_encoder->PIDGet());
}

void TestEncoder::Start() {
	m_encoder->Start();
}

void TestEncoder::Stop() {
	m_encoder->Stop();
}

void TestEncoder::Reset() {
	m_encoder->Reset();
}

void TestEncoder::SetMaxPeriod(double maxPeriod) {
	m_encoder->SetMaxPeriod(maxPeriod);
}

void TestEncoder::SetMinRate(double minRate) {
	m_encoder->SetMinRate(minRate);
}

void TestEncoder::SetDistancePerPulse(double input) {
	m_distancePerPulse = input;
	m_encoder->SetDistancePerPulse(input);
}

void TestEncoder::SetReverseDirection(bool reverseDirection) {
	m_encoder->SetReverseDirection(reverseDirection);
}

void TestEncoder::SetPIDSourceParameter(Encoder::PIDSourceParameter pidSource) {
	m_encoder->SetPIDSourceParameter(pidSource);
}

double TestEncoder::GetDistancePerPulse() {
	return m_distancePerPulse;
}

void TestEncoder::Log(const char *key, int value) {
	SmartDashboard::PutNumber(GetName() + std::string(" ") + std::string(key), value);
}

void TestEncoder::Log(const char *key, double value) {
	SmartDashboard::PutNumber(GetName() + std::string(" ") + std::string(key), value);
}

void TestEncoder::LogBool(const char *key, bool value) {
	SmartDashboard::PutBoolean(GetName() + std::string(" ") + std::string(key), value);
}



/**
 * Common initialization code called by all constructors.
 *
 * Note that the Victor uses the following bounds for PWM values.  These values were determined
 * empirically and optimized for the Victor 888. These values should work reasonably well for
 * Victor 884 controllers as well but if users experience issues such as asymmetric behavior around
 * the deadband or inability to saturate the controller in either direction, calibration is recommended.
 * The calibration procedure can be found in the Victor 884 User Manual available from IFI.
 * 
 *   - 206 = full "forward"
 *   - 131 = the "high end" of the deadband range
 *   - 128 = center of the deadband range (off)
 *   - 125 = the "low end" of the deadband range
 *   - 56 = full "reverse"
 */
void ReversedVictor::InitVictor() {
	// TODO: compute the appropriate values based on digital loop timing
	SetBounds(206, 131, 128, 125, 56);
	SetPeriodMultiplier(kPeriodMultiplier_2X);
	SetRaw(m_centerPwm);

	nUsageReporting::report(nUsageReporting::kResourceType_Victor, GetChannel(), GetModuleNumber() - 1);
}

/**
 * Constructor that assumes the default digital module.
 * 
 * @param channel The PWM channel on the digital module that the Victor is attached to.
 */
ReversedVictor::ReversedVictor(UINT32 channel) : SafePWM(channel)
{
	InitVictor();
}

/**
 * Constructor that specifies the digital module.
 * 
 * @param moduleNumber The digital module (1 or 2).
 * @param channel The PWM channel on the digital module that the Victor is attached to (1..10).
 */
ReversedVictor::ReversedVictor(UINT8 moduleNumber, UINT32 channel) : SafePWM(moduleNumber, channel)
{
	InitVictor();
}

ReversedVictor::~ReversedVictor()
{
}

/**
 * Set the PWM value.  
 * 
 * The PWM value is set using a range of -1.0 to 1.0, appropriately
 * scaling the value for the FPGA.
 * 
 * @param speed The speed value between -1.0 and 1.0 to set.
 * @param syncGroup Unused interface.
 */
void ReversedVictor::Set(float speed, UINT8 syncGroup)
{
	SetSpeed(-speed);
}

/**
 * Get the recently set value of the PWM.
 * 
 * @return The most recently set value for the PWM between -1.0 and 1.0.
 */
float ReversedVictor::Get()
{
	return -GetSpeed();
}

/**
 * Common interface for disabling a motor.
 */
void ReversedVictor::Disable()
{
	SetRaw(kPwmDisabled);
}

/**
 * Write out the PID value as seen in the PIDOutput base object.
 * 
 * @param output Write out the PWM value as was found in the PIDController
 */
void ReversedVictor::PIDWrite(float output) 
{
	Set(-output);
}
