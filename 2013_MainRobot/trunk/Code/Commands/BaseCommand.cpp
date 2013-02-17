#include "BaseCommand.h"

SimpleCommand::SimpleCommand(const char *name, bool valueToReturnForIsFinished) :
		Command(name),
		m_valueToReturnForIsFinished(valueToReturnForIsFinished) {
	// empty
}
	
SimpleCommand::~SimpleCommand() {
	// empty
}

void SimpleCommand::Initialize() {
	// empty
}

bool SimpleCommand::IsFinished() {
	return m_valueToReturnForIsFinished;
}

void SimpleCommand::End() {
	// empty
}

void SimpleCommand::Interrupted() {
	//empty
}
	
