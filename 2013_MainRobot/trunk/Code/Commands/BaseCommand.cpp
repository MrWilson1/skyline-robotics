#include "BaseCommand.h"

SimpleCommand::SimpleCommand(const char *name, bool doesCommandRunEndlessly) :
		Command(name),
		m_doesCommandRunEndlessly(doesCommandRunEndlessly) {
	// empty
}
	
SimpleCommand::~SimpleCommand() {
	// empty
}

void SimpleCommand::Initialize() {
	// empty
}

bool SimpleCommand::IsFinished() {
	return !m_doesCommandRunEndlessly;
}

void SimpleCommand::End() {
	// empty
}

void SimpleCommand::Interrupted() {
	//empty
}
	
