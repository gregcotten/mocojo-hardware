#include "LongBuffer.h"

long _buffer[1];
int _bufferSize;
int _currentPosition;
int _currentBufferPosition;
int _amountFresh;

LongBuffer::LongBuffer(int bufferSize){
	_bufferSize = bufferSize;
	_buffer[_bufferSize];
	reset();
}

int LongBuffer::amountFresh(){
	return _amountFresh;
}

bool LongBuffer::isFull(){
	return _amountFresh == _bufferSize;
}

void LongBuffer::reset(){
	_currentPosition = 0;
	_currentBufferPosition = 0;
	_amountFresh = 0;
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

long LongBuffer::peek(){
	return _buffer[_currentPosition];
}

