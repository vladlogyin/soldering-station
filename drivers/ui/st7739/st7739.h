#ifndef ST7735_H_INCLUDED
#define ST7735_H_INCLUDED


#include "../display.h"

#include <cstdint>

class st7739 : public display
{
public:
	st7739();
	virtual void init() override;
	virtual void update() override;
private:
};

#endif // ST7735_H_INCLUDED
