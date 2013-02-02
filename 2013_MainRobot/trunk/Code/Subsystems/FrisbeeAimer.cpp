#include "FrisbeeAimer.h"


BaseFrisbeeAimer::BaseFrisbeeAimer(const char *name) :
		BaseSubsystem(name) {
	// Empty
}

BaseFrisbeeAimer::~BaseFrisbeeAimer() {
	// Empty
}

Tracking::Offset::Offset() :
		XOffset(0),
		YOffset(0) {
	// Empty
}

Tracking::Offset::Offset(float x, float y) :
		XOffset(x), 
		YOffset(y) {
	// Empty
}

Tracking::Target::Target() :
		Type(Tracking::Unknown),
		ShooterOffset(),
		DistanceInInches(0) {
	// empty
}

Tracking::Target::Target(Tracking::TargetType type, Tracking::Offset shooterOffset, double distanceInInches) :
		Type(type),
		ShooterOffset(shooterOffset.XOffset, shooterOffset.YOffset),
		DistanceInInches(distanceInInches) {
	// empty
}

float Tracking::FindTargetMagnitude(Target target) {
	float x = target.ShooterOffset.XOffset;
	float y = target.ShooterOffset.YOffset;
	return Tools::FindMagnitude(x, y);
}
		

VisionTablesFrisbeeAimer::VisionTablesFrisbeeAimer(std::string tableName) :
		BaseFrisbeeAimer("VisionTablesFrisbeeAimer"),
		m_tableName(tableName) {
	m_visionTable = NetworkTable::GetTable(m_tableName);
}

VisionTablesFrisbeeAimer::~VisionTablesFrisbeeAimer() {
	// empty
}
	
Tracking::TargetMap VisionTablesFrisbeeAimer::GetAllTargets() {
	Tracking::TargetMap targetMap;
	targetMap[Tracking::High] = GetHighTarget();
	targetMap[Tracking::MediumLeft] = GetMediumLeftTarget();
	targetMap[Tracking::MediumRight] = GetMediumRightTarget();
	targetMap[Tracking::Low] = GetLowTarget();
	return targetMap;
}

Tracking::Target VisionTablesFrisbeeAimer::GetHighTarget() {
	return GetTarget("high");
}

Tracking::Target VisionTablesFrisbeeAimer::GetMediumLeftTarget() {
	return GetTarget("medium_left");
}

Tracking::Target VisionTablesFrisbeeAimer::GetMediumRightTarget() {
	return GetTarget("medium_right");
}

Tracking::Target VisionTablesFrisbeeAimer::GetLowTarget() {
	return GetTarget("low");
}

Tracking::Target VisionTablesFrisbeeAimer::GetClosestTargetByDistance() {
	return GetTarget("best_distance");
}

Tracking::Target VisionTablesFrisbeeAimer::GetClosestTargetByOffset() {
	return GetTarget("best_offset");
}
	
Tracking::Target VisionTablesFrisbeeAimer::GetTarget(std::string key) {
	Tracking::Target target;
	std::string rectType = m_visionTable->GetString(key + "_rect_type");
	
	// A bit ugly, but whatever.
	if (rectType == "none") {
		target.Type = Tracking::None;
	} else if (rectType == "unknown"){
		target.Type = Tracking::Unknown;
	} else if (rectType == "test") {
		target.Type = Tracking::Test;
	} else if (rectType == "Low") {
		target.Type = Tracking::Low;
	} else if (rectType == "medium_left") {
		target.Type = Tracking::MediumLeft;
	} else if (rectType == "medium_right") {
		target.Type = Tracking::MediumRight;
	} else if (rectType == "high") {
		target.Type = Tracking::High;
	} else if (rectType == "pyramid") {
		target.Type = Tracking::Pyramid;
	} else {
		target.Type = Tracking::Unknown;
	}
	
	target.ShooterOffset.XOffset = m_visionTable->GetNumber(key + "_offset_x");
	target.ShooterOffset.YOffset = m_visionTable->GetNumber(key + "_offset_y");
	target.DistanceInInches = m_visionTable->GetNumber(key + "_distance");
	return target;
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
	
	float high = table->GetNumber("high_distance");
	float test = table->GetNumber("test_distance");
	
	SmartDashboard::PutNumber("high distance", high/12);
	SmartDashboard::PutNumber("test distance", test/12);
}
