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

/**
 * This is stupidly implemented. I'll probably revamp it later.
 */
Tracking::Target VisionTablesFrisbeeAimer::GetClosestTargetByDistance() {
	Tracking::TargetMap targets = GetAllTargets();
	typedef Tracking::TargetMap::iterator TargetIter;
	Tracking::Target bestTarget;
	float bestDistance = -1;
	for (TargetIter targetIter = targets.begin(); targetIter != targets.end(); targetIter++) {
		Tracking::Target currentTarget = targetIter->second;
		if (currentTarget.DistanceInInches == 0) {
			// If the target is an invalid one, skip it
			continue;
		}
		if (bestDistance < 0) {
			// If there currently is no best previous target, then by default,
			// make this target the best one.
			bestDistance = currentTarget.DistanceInInches;
			bestTarget = currentTarget;
			continue;
		}
		if (currentTarget.DistanceInInches < bestDistance) {
			// If this is better then the previous target, replace it.
			bestDistance = currentTarget.DistanceInInches;
			bestTarget = currentTarget;
			continue;
		}
	}
	return bestTarget;
}

/**
 * This is stupidly implemented. I'll probably revamp it later.
 */
Tracking::Target VisionTablesFrisbeeAimer::GetClosestTargetByOffset() {
	Tracking::TargetMap targets = GetAllTargets();
	typedef Tracking::TargetMap::iterator TargetIter;
	Tracking::Target bestTarget;
	float bestOffsetMagnitude = -1;
	
	for (TargetIter targetIter = targets.begin(); targetIter != targets.end(); targetIter++) {
		Tracking::Target currentTarget = targetIter->second;
		float offsetMagnitude = Tracking::FindTargetMagnitude(currentTarget);
		
		if (offsetMagnitude == 0) {
			// If the target is an invalid one, skip it
			continue;
		}
		if (bestOffsetMagnitude < 0) {
			// If there currently is no best previous target, then by default,
			// make this target the best one.
			bestOffsetMagnitude = offsetMagnitude;
			bestTarget = currentTarget;
			continue;
		}
		if (offsetMagnitude < bestOffsetMagnitude) {
			// If this is better then the previous target, replace it.
			bestOffsetMagnitude = offsetMagnitude;
			bestTarget = currentTarget;
			continue;
		}
	}
	return bestTarget;
}
	
Tracking::Target VisionTablesFrisbeeAimer::GetTarget(std::string key) {
	Tracking::Target target;
	target.Type = Tracking::MediumLeft;
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
