#ifndef ENCODER_H_INCLUDED
#define ENCODER_H_INCLUDED

#include <cstdint>

class encoder {
public:


	void init();
	int getDelta();
	bool poll();
	static encoder& getInstance();
	int updateEncoder();
private:
	encoder() = default;
	~encoder() = default;
	encoder(encoder const&) = delete;
	void operator=(encoder const&) = delete;

	uint8_t getState();

	int _delta;
	int _lastState;
	const static uint8_t _nextStates[4];
	const static uint8_t _previousStates[4];
};


#endif // ENCODER_H_INCLUDED
