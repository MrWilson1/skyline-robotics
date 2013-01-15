#include "BaseOI.h"

BaseOI::BaseOI(const char *name) :
		m_name(name) {
	// empty
}

BaseOI::~BaseOI() {
	// empty
}

std::string BaseOI::GetName() {
	return m_name;
}
