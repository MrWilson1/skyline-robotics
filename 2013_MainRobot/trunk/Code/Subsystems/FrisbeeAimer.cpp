#include "FrisbeeAimer.h"


BaseFrisbeeAimer::BaseFrisbeeAimer(const char *name) :
		BaseSubsystem(name) {
	// Empty
}

BaseFrisbeeAimer::~BaseFrisbeeAimer() {
	// Empty
}

Offset::Offset(float x, float y) :
		XOffset(x), 
		YOffset(y) {
	// Empty
}


TestAimer::TestAimer(const char *name) :
		BaseSubsystem(name) {
	// empty
}

TestAimer::~TestAimer() {
	// empty
}

void TestAimer::Run() {
	NetworkTable *table = NetworkTable::GetTable("VisionTable");
	NumberArray high_array;
	NumberArray medium1_array;
	NumberArray medium2_array;
	NumberArray low_array;
	
	table->RetrieveValue("high", high_array);
	table->RetrieveValue("medium1", medium1_array);
	table->RetrieveValue("medium2", medium2_array);
	table->RetrieveValue("low", low_array);
	
	SmartDashboard::PutNumber("high distance", high_array.get(2));
	SmartDashboard::PutNumber("medium1 distance", medium1_array.get(2));
	SmartDashboard::PutNumber("medium2 distance", medium2_array.get(2));
	SmartDashboard::PutNumber("low distance", low_array.get(2));
}
