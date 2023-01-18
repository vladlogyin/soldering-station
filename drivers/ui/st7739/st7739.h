#ifndef ST7735_H_INCLUDED
#define ST7735_H_INCLUDED


#include "../display.h"

#include <cstdint>

class st7739 : public display
{
public:
	//st7739();
	virtual void init() override;
	virtual void update() override;
	void setScroll(uint16_t scroll);
	void displayBin(uint16_t x, uint16_t y, uint32_t num);
	void displayInt(uint16_t x, uint16_t y, int32_t num, int decimals=0);

	static st7739& getInstance();

	void setTargetTemp(const float temp);
	void setCurrentTemp(const float temp);
	void setCurrentDuty(const float duty);


private:

	bool _invalidated;
	int _targetTemp;
	int _currentTemp;
	int _currentDuty;

	void invalidate();

	st7739() = default;
	~st7739() = default;
	st7739(st7739 const&) = delete;
	void operator=(st7739 const&) = delete;
};

#endif // ST7735_H_INCLUDED
