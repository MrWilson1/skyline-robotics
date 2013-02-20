#ifndef DRIVE_COMPONENTS_H
#define DRIVE_COMPONENTS_H

#include "WPILib.h"
#include "../../Misc/Tools.h"

static const std::string kSpeedLabel = std::string("Speed");
static const std::string kDirectionLabel = std::string("Direction Multiplier (1 or -1)");

class Tread : public PIDOutput, public LiveWindowSendable, public ITableListener {
public:
	/**
	 * \brief Specifies the direction the motors should spin to move forwards
	 */
	enum Direction {
		kForward = 1,
		kReverse = -1
	};
	Tread(SpeedController *motor);
	Tread(SpeedController *front, SpeedController *back);
	Tread(std::vector<SpeedController*> motors);
	virtual ~Tread();
	
	
	void SetSpeed(double speed);
	Direction GetDirection();
	void SetDirection(Direction direction);
	double GetSpeed();
	
	// Required by PIDOutput
	void PIDWrite(float output);
	
	// Required by Sendable, which LiveWindowSendable inherits from.
	/**
	 * \brief Initializes a table representing this object.
	 * 
	 * \param[in] subtable A pointer to an empty table which this method
	 * must initialize to represent this object.
	 */
	void InitTable(ITable *subtable);
	ITable* GetTable();
	std::string GetSmartDashboardType();
	
	// Required by LiveWindowSendable
	void UpdateTable();
	void StartLiveWindowMode();
	void StopLiveWindowMode();
	
	// Required by ITableListener
	/**
	 * \brief This function is called when the table representing this object
	 * is updated.
	 * 
	 * \warning if a new value is added to the NetworkTable inside this method,
	 * this method will be called again, leading to potentially recursive code.
	 * 
	 * \param[in] source A pointer to the table representing this object
	 * \param[in] key The string with the name of this key
	 * \param[in] value The value of the key
	 * \param[in] isNew True if the key-value pair was added to the table. False 
	 * 		      if the pair was only updated.
	 */
	void ValueChanged(
			ITable *source, 
			const std::string &key, 
			EntryValue value, 
			bool isNew);
private:
	ITable *m_table;
	double m_speed;
	Direction m_direction;
	std::vector<SpeedController*> m_motors;
};

#endif
