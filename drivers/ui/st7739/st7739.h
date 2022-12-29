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
	void setWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
};

#endif // ST7735_H_INCLUDED
