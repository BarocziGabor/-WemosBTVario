#ifndef TIMEDFUNC
#define TIMEDFUNC
void timedFunc(uint32_t *prevTime, uint16_t delayInMillisecs, void(*func)())
{
	if (millis() - *prevTime > delayInMillisecs) {
		func();
		*prevTime = millis();
	}
}
#endif