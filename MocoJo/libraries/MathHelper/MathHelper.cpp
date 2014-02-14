#include <MathHelper.h>

//All send/receive methods use MSB byte order
float MathHelper::fromIntTo01(int value, float maxValue){
	return (float)value / maxValue;
	
}

int MathHelper::from01ToInt(float value, int maxValue){
	return (int)(value * (float)maxValue);
}

float MathHelper::absvalue(float value){
	if(value < 0.0){
		return -1.0*value;
	}
	return value;
}

float MathHelper::clamp(float value, float minValue, float maxValue){
	if(value < minValue){
		return minValue;
	}
	else if(value > maxValue){
		return maxValue;
	}

	return value;
}

int MathHelper::clamp(int value, int minValue, int maxValue){
	if(value < minValue){
		return minValue;
	}
	else if(value > maxValue){
		return maxValue;
	}

	return value;
}
