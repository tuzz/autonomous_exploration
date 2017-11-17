#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

class Environment {
public:
	Environment(float width, float height);
	float width, height;
	void addObstacle(Polygon p);
	std::list<Polygon> obstacles;
	bool bounds(Vertex v);
	void addBeacon(Polygon p);
	std::list<Polygon> beacons;
};

Environment::Environment(float width, float height) {
	this->width = width;
	this->height = height;

	//Create virtual obstacles for the boundaries.
	//These overlap to prevent corner collision detection.
	Polygon top;
	top.addVertex(Vertex(-0.025f * width, height));
	top.addVertex(Vertex(-0.025f * width, height * 1.025f));
	top.addVertex(Vertex(width * 1.025f, height * 1.025f));
	top.addVertex(Vertex(width * 1.025f, height));
	addObstacle(top);

	Polygon right;
	right.addVertex(Vertex(width, height * 1.025f));
	right.addVertex(Vertex(width * 1.025f, height * 1.025f));
	right.addVertex(Vertex(width * 1.025f, -0.025f * height));
	right.addVertex(Vertex(width, -0.025f * height));
	addObstacle(right);

	Polygon bottom;
	bottom.addVertex(Vertex(-0.025f * width, -0.025f * height));
	bottom.addVertex(Vertex(-0.025f * width, 0));
	bottom.addVertex(Vertex(width * 1.025f, 0));
	bottom.addVertex(Vertex(width * 1.025f, -0.025f * height));
	addObstacle(bottom);

	Polygon left;
	left.addVertex(Vertex(-0.025f * width, -0.025f * height));
	left.addVertex(Vertex(0, -0.025f * height));
	left.addVertex(Vertex(0, height * 1.025f));
	left.addVertex(Vertex(-0.025f * width, height * 1.025f));
	addObstacle(left);
}

void Environment::addObstacle(Polygon p) {
	obstacles.push_back(p);
}

void Environment::addBeacon(Polygon p) {
	obstacles.push_back(p);
	beacons.push_back(p);
}

bool Environment::bounds(Vertex v) {
	if (v.x < 0 || v.x > width) return false;
	if (v.y < 0 || v.y > height) return false;
	return true;
}

#endif