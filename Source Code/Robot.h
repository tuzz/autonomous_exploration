#ifndef ROBOT_H
#define ROBOT_H

#include "Vertex.h"
#include <Math.h>
#include <list>
#include <time.h>
#define PI 3.14159265

class Robot {
public:
	Robot(float width, float height, Vertex location, float moveRate, float turnRate, float lidarRate, float moveNoise, float turnNoise, float lidarNoise, Environment *e);
	float width, height;
	Vertex location;
	float angle, lidarAngle;
	void updateVertices();
	Vertex frontLeft, frontRight, backLeft, backRight;
	float moveRate, turnRate, lidarRate;
	void forward(float amount);
	void backward(float amount);
	void right(float angle);
	void left(float angle);
	void updateCollision();
	bool collision;
	float moveNoise(float amount);
	float turnNoise(float angle);
	Environment *e;
	void lidar(float angle);
	bool lidarBeacon();
	float lidarNoise(float angle);
	void updateLidar();
	Vertex minVertex;
	Vertex lidarIntercept(Vertex a, Vertex b);
	float distance(Vertex v);
	float lidarDistance;
	float gaussianRandom(float mean, float variance);
	float mNoise, tNoise, lNoise;
	Vertex startLocation;
	void restore();
};

/*
moveRate: How many units with respect to the environment, the robot moves per second.
turnRate: How many degrees the robot turns per second.
lidarRate: How many degrees the lidar turns per second.
*/
Robot::Robot(float width, float height, Vertex location, float moveRate, float turnRate, float lidarRate, float moveNoise, float turnNoise, float lidarNoise, Environment *e) {
	this->width = width;
	this->height = height;
	this->startLocation = location;
	this->location = location;
	this->moveRate = moveRate;
	this->turnRate = turnRate;
	this->lidarRate = lidarRate;
	this->mNoise = moveNoise;;
	this->tNoise = turnNoise;
	this->lNoise = lidarNoise;
	this->e = e;
	angle = 0;
	lidarAngle = 0;
	lidarDistance = (e->width > e->height) ? e->width * 2 : e->height * 2;
	srand((int)time(NULL));

	updateVertices();
}

//Resets the robot to its original state for a new test.
void Robot::restore() {
	location = startLocation;
	angle = 0;
	lidarAngle = 0;
	lidarDistance = (e->width > e->height) ? e->width * 2 : e->height * 2;
	updateVertices();
}

//Updates the vertices of the robot based on its location and rotation.
void Robot::updateVertices() {
	//Calculate width and height divided by 2.
	float y2 = width / 2;
	float x2 = height / 2;

	//Convert degrees to radians.
	float radians = angle * (float)PI / 180;

	//Rotate each point about the axis and translate it (rotation matrix).
	frontRight.x = x2 * cos(radians) + y2 * sin(radians) + location.x;
	frontRight.y = x2 * sin(radians) - y2 * cos(radians) + location.y;
	frontLeft.x = x2 * cos(radians) - y2 * sin(radians) + location.x;
	frontLeft.y = x2 * sin(radians) + y2 * cos(radians) + location.y;
	backLeft.x = -x2 * cos(radians) - y2 * sin(radians) + location.x;
	backLeft.y = -x2 * sin(radians) + y2 * cos(radians) + location.y;
	backRight.x = -x2 * cos(radians) + y2 * sin(radians) + location.x;
	backRight.y = -x2 * sin(radians) - y2 * cos(radians) + location.y;
}

//Moves the robot unless there will be a collision.
void Robot::forward(float amount) {
	amount = gaussianRandom(amount, amount * mNoise);
	float radians = angle * (float)PI / 180;
	location.x += amount * cos(radians);
	location.y += amount * sin(radians);
	updateVertices();
	updateCollision();

	if (collision) {
		location.x -= amount * cos(radians);
		location.y -= amount * sin(radians);
	}
	updateVertices();
	updateCollision();
}

//Moves the robot unless there will be a collision.
void Robot::backward(float amount) {
	amount = gaussianRandom(amount, amount * mNoise);
	float radians = angle * (float)PI / 180;
	location.x -= amount * cos(radians);
	location.y -= amount * sin(radians);
	updateVertices();
	updateCollision();

	if (collision) {
		location.x += amount * cos(radians);
		location.y += amount * sin(radians);
	}
	updateVertices();
	updateCollision();
}

//Turns the robot unless there will be a collision.
void Robot::left(float angle) {
	angle = gaussianRandom(angle, angle * tNoise);
	this->angle += angle;
	if (this->angle >= 360) this->angle -= 360;
	updateVertices();
	updateCollision();

	if (collision) {
		this->angle -= angle;
		if (this->angle < 0) this->angle += 360;
	}
	updateVertices();
	updateCollision();
}

//Turns the robot unless there will be a collision.
void Robot::right(float angle) {
	angle = gaussianRandom(angle, angle * tNoise);
	this->angle -= angle;
	if (this->angle < 0) this->angle += 360;
	updateVertices();
	updateCollision();

	if (collision) {
		this->angle += angle;
		if (this->angle >= 360) this->angle -= 360;
	}
	updateVertices();
	updateCollision();
}

//Checks if the robot is coliding with the boundaries or any polygon.
void Robot::updateCollision() {
	collision = false;

	if (!e->bounds(frontLeft)) { collision = true; return; }
	if (!e->bounds(frontRight)) { collision = true; return; }
	if (!e->bounds(backLeft)) { collision = true; return; }
	if (!e->bounds(backRight)) { collision = true; return; }

	Polygon robotPolygon;
	robotPolygon.addVertex(frontLeft);
	robotPolygon.addVertex(frontRight);
	robotPolygon.addVertex(backRight);
	robotPolygon.addVertex(backLeft);

	for (std::list<Polygon>::iterator i = e->obstacles.begin(); i != e->obstacles.end(); i++) {
		if (i->overlaps(robotPolygon)) {
			collision = true;
			return;
		}
	}
}

void Robot::lidar(float angle) {
	angle = gaussianRandom(angle, angle * lNoise);
	lidarAngle += angle;
	if (lidarAngle >= 360) lidarAngle -= 360;

	updateLidar();
}

//Updates lidarDistance with the closest point the lidar intercepts.
void Robot::updateLidar() {
	//Loop through the vertices of polygons trying to find the nearest vertex.
	float curDistance, minDistance = (e->width > e->height) ? e->width * 2 : e->height * 2;
	Vertex curVertex;
	for (std::list<Polygon>::iterator i = e->obstacles.begin(); i != e->obstacles.end(); i++) {
		Vertex last = i->vertices.back();
		for (std::list<Vertex>::iterator j = i->vertices.begin(); j != i->vertices.end(); j++) {
			curVertex = lidarIntercept(last, *j);
			last = *j;
			if (curVertex.x != -1) {
				curDistance = distance(curVertex);
				if (curDistance < minDistance) {
					minDistance = curDistance;
					minVertex = i->vertices.front();
				}
			}
		}
	}

	//Updates the lidarDistance variable.
	lidarDistance = minDistance;
}

//Loops through each vertex of each beacon, checks if the lidar intercept is for that beacon.
bool Robot::lidarBeacon() {
	for (std::list<Polygon>::iterator i = e->beacons.begin(); i != e->beacons.end(); i++)
		for (std::list<Vertex>::iterator j = i->vertices.begin(); j != i->vertices.end(); j++)
			if (minVertex.x == j->x && minVertex.y == j->y)
				return true;
	return false;
}

//Returns intercept if the lidar at the current angle, intersects the line between a and b.
//Otherwise -1, -1
Vertex Robot::lidarIntercept(Vertex a, Vertex b) {
	float rad = lidarAngle * (float)PI / 180;

	//Convert the lines to form: Ax + By = C
	float A1 = -tan(rad);
	float B1 = 1;
	float C1 = A1 * location.x + B1 * location.y;

	float A2 = b.y - a.y;
	float B2 = a.x - b.x;
	float C2 = A2 * a.x + B2 * a.y;

	//Check if the lines are parallel.
	float det = A1 * B2 - A2 * B1;
	if (det == 0)
		return Vertex(-1, -1);
	
	float x = (B2 * C1 - B1 * C2) / det;
	float y = (A1 * C2 - A2 * C1) / det;

	//Get the min and max values.
	float minX = (a.x < b.x) ? a.x : b.x;
	float maxX = (a.x < b.x) ? b.x : a.x;
	float minY = (a.y < b.y) ? a.y : b.y;
	float maxY = (a.y < b.y) ? b.y : a.y;

	//Check if the intersection is part of the line strip.
	if (minX - 0.0001 <= x && x <= maxX + 0.0001 && minY - 0.0001 <= y && y <= maxY + 0.0001) {
		//Check if the vertex is on the lidar side of the robot.
		if (x < location.x && (lidarAngle < 90 || lidarAngle > 270))
			return Vertex(-1, -1);
		if (x > location.x && (lidarAngle < 270 && lidarAngle > 90))
			return Vertex(-1, -1);

		return Vertex(x, y);
	}

	//Not part of line strip.
	return Vertex(-1, -1);
}

//Calculate the distance from the centre of the robot to a vertex.
float Robot::distance(Vertex v) {
	//Find the lengths of x and y.
	float dx = v.x - location.x;
	float dy = v.y - location.y;

	//Calculate the hypotenuse.
	float hypotenuse = sqrt(dx * dx + dy * dy);

	return hypotenuse;
}

//Uses the box-muller method for obtaining gaussian random numbers.
float Robot::gaussianRandom(float mean, float variance) {
	//Efficiency improvement when there's no noise.
	if (variance == 0) return mean;
	
	float sd = sqrt(variance);
	float x1, x2, w, y1;
	static float y2;
	static bool set = false;

	//Reuse previous y1 value (efficient).
	if (set) {
		y1 = y2;
		set = false;
	}
	else {
		do {
			x1 = (float)(2.0 * rand() / RAND_MAX - 1.0);
			x2 = (float)(2.0 * rand() / RAND_MAX - 1.0);
			w = x1 * x1 + x2 * x2;
		}
		while (w >= 1.0);

		w = (float)sqrt((-2.0 * log(w)) / w);
		y1 = x1 * w;
		y2 = x2 * w;
		set = true;
	}

	//box-muller equation with scalar.
	return mean + y1 * sd;
}

#endif