#ifndef VERTEX_H
#define VERTEX_H

class Vertex {
public:
	Vertex();
	Vertex(float x, float y);

	//Set uses lt-comparison.
	bool operator<(const Vertex &other) const {
		if (x == other.x)
			return (y < other.y);
		return (x < other.x);
	}

	bool operator==(const Vertex &other) {
		return (x == other.x && y == other.y);
	}

	float x, y;
};

Vertex::Vertex() {
	this->x = 0;
	this->y = 0;
}

Vertex::Vertex(float x, float y) {
	this->x = x;
	this->y = y;
}

#endif