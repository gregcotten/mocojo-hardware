#ifndef LongBuffer_h
#define LongBuffer_h

#include <WProgram.h>
#include <WString.h>

class LongBuffer{
	public:
		LongBuffer(int bufferSize);
		bool isFull();
		int amountFresh();
		void addLong(long data);
		long nextLong();
		long peek();
		void reset();
	private:
		long _buffer[];

		int _bufferSize;
		int _currentPosition;
		int _currentBufferPosition;
		int _amountFresh;

		
};
#endif