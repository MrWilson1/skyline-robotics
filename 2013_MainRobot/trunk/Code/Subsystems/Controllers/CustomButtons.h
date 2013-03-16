#ifndef CUSTOM_BUTTONS_H
#define CUSTOM_BUTTONS_H

#include "WPILib.h"

namespace CustomButton {

class PressTwo : public Button {
public:
	PressTwo(Button *button1, Button *button2);
	~PressTwo();
	
	bool Get();
	
private:
	Button *m_button1;
	Button *m_button2;
};

}

#endif
