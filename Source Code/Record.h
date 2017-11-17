#ifndef RECORD_H
#define RECORD_H

class Record {
public:
	float x, y, b, l, d, xV, yV;
	Record(float x, float y, float b, float l, float d, float xV, float yV) {
		this->x = x;
		this->y = y;
		this->b = b;
		this->l = l;
		this->d = d;
		this->xV = xV;
		this->yV = yV;
	}
};

#endif