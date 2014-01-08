#ifndef MathHelper_h
#define MathHelper_h

namespace MathHelper {
	float fromIntTo01(int value, float maxValue);
	int from01ToInt(float value, int maxValue);
	float clamp(float value, float minValue, float maxValue);
	int clamp(int value, int minValue, int maxValue);

}

#endif
