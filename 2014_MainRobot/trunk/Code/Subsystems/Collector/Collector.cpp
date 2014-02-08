#include "Collector.h"

/* Roflcopter
 
   ROFL:ROFL:LOL:ROFL:ROFL
              ^
  L     /------------- 
  O ====          []  \
  L     \              \
         \______________]
            I      I
        ----------------/

 */

Collector::Collector(Victor *motor, Solenoid *piston, Compressor *compressor, DigitalInput *limitSwitch) {
	m_motor = motor;
	m_piston = piston;
	m_compressor = compressor;
	m_limitSwitch = limitSwitch;
}

bool Collector::GetLimitSwitch() {
	return m_limitSwitch->Get();
}

void Collector::SpinInwards() {
	m_motor->Set(1.0);
}

void Collector::SpinOutwards() {
	m_motor->Set(-1.0);
}

void Collector::SpinStop() {
	m_motor->Set(0.0);
}

void Collector::PistonUp() {
	m_piston->Set(true);
}

void Collector::PistonDown() {
	m_piston->Set(false);
}

void Collector::PistonUpAndDown() {
	PistonUp();
	PistonDown();
}