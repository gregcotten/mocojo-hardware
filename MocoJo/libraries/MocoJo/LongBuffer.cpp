#include "LongBuffer.h"

long _buffer[1];
int _bufferSize;
int _currentPosition = 0;
int _currentBufferPosition = 0;
int _amountFresh = 0;

LongBuffer::LongBuffer(int bufferSize){
	_bufferSize = bufferSize;
	_buffer[_bufferSize];
}

int LongBuffer::amountFresh(){
	return _amountFresh;
}

void LongBuffer::addLong(long data){
	_buffer[_currentBufferPosition] = data;
	_currentBufferPosition = (_currentBufferPosition + 1) % _bufferSize;
	_amountFresh++;
}

long LongBuffer::nextLong(){
	long data = _buffer[_currentPosition];
	_currentPosition == (_currentPosition + 1) % _bufferSize;
	_amountFresh--;
	return data;
}

