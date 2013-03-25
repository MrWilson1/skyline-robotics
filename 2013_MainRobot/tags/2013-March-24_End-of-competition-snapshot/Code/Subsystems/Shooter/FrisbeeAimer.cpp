#include "FrisbeeAimer.h"


FrisbeeAimer::Base::Base(const char *name) :
		BaseSubsystem(name) {
	// Empty
}

FrisbeeAimer::Base::~Base() {
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

FrisbeeAimer::VisionTables::VisionTables() :
		Base("VisionTables"),
		m_tableName("VisionTable") {
	m_visionTable = NetworkTable::GetTable(m_tableName);
}

FrisbeeAimer::VisionTables::VisionTables(std::string tableName) :
		Base("VisionTables"),
		m_tableName(tableName) {
	m_visionTable = NetworkTable::GetTable(m_tableName);
}

FrisbeeAimer::VisionTables::~VisionTables() {
	// empty
}
	
Tracking::TargetMap FrisbeeAimer::VisionTables::GetAllTargets() {
	Tracking::TargetMap targetMap;
	targetMap[Tracking::High] = GetHighTarget();
	targetMap[Tracking::MiddleLeft] = GetMiddleLeftTarget();
	targetMap[Tracking::MiddleRight] = GetMiddleRightTarget();
	targetMap[Tracking::Low] = GetLowTarget();
	return targetMap;
}

Tracking::Target FrisbeeAimer::VisionTables::GetHighTarget() {
	return GetTarget("high");
}

Tracking::Target FrisbeeAimer::VisionTables::GetMiddleLeftTarget() {
	return GetTarget("middle_left");
}

Tracking::Target FrisbeeAimer::VisionTables::GetMiddleRightTarget() {
	return GetTarget("middle_right");
}

Tracking::Target FrisbeeAimer::VisionTables::GetLowTarget() {
	return GetTarget("low");
}

Tracking::Target FrisbeeAimer::VisionTables::GetClosestTargetByDistance() {
	return GetTarget("best_distance");
}

Tracking::Target FrisbeeAimer::VisionTables::GetClosestTargetByOffset() {
	return GetTarget("best_offset");
}
	
Tracking::Target FrisbeeAimer::VisionTables::GetTarget(std::string key) {
	Tracking::Target target;
	std::string rectType = m_visionTable->GetString(key + "_rect_type");
	
	// A bit ugly, but whatever.
	if (rectType == "none") {
		target.Type = Tracking::None;
	} else if (rectType == "unknown"){
		target.Type = Tracking::Unknown;
	} else if (rectType == "test") {
		target.Type = Tracking::Test;
	} else if (rectType == "low") {
		target.Type = Tracking::Low;
	} else if (rectType == "middle_left") {
		target.Type = Tracking::MiddleLeft;
	} else if (rectType == "middle_right") {
		target.Type = Tracking::MiddleRight;
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



FrisbeeAimer::Test::Test(const char *name) :
		BaseSubsystem(name) {
	// empty
}

FrisbeeAimer::Test::~Test() {
	// empty
}

void FrisbeeAimer::Test::Run() {
	NetworkTable *table = NetworkTable::GetTable("VisionTable");
	
	float high = table->GetNumber("high_distance");
	float test = table->GetNumber("test_distance");
	
	SmartDashboard::PutNumber("high distance", high/12);
	SmartDashboard::PutNumber("test distance", test/12);
}
