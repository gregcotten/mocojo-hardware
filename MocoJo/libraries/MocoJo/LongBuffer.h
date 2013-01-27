#ifndef LongBuffer_h
#define LongBuffer_h

#include <WProgram.h>
#include <WString.h>

class LongBuffer{
	public:
		LongBuffer(int bufferSize);
		int amountFresh();
		int addLong();
		int nextLong();
	private:
		long _buffer[];

		int _bufferSize;
		int _currentPosition;
		int _currentBufferPosition;
		int _amountFresh;

		
};
#endif