#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include <vector>
#include <set>
#include <algorithm>
#include <Math.h>
#define PI 3.14159265
#include "Vertex.h"
#include "Record.h"
#include "Grid.h"
using namespace std;

class Behaviour {
public:
	Behaviour(Robot *r, double startGran, double minGran, double split, int strategy);
	Robot *r;
	void nextMove(float n);
	void nextLidar(float n);
	void forward(float amount);
	void backward(float amount);
	void left(float angle);
	void right(float angle);
	void lidar(float angle);
	Vertex getVertex(float x, float y, float l, float d);
	set<Vertex> overlay(Vertex v, float a);
	float collision(int move, float amount);
	void runStrategy(float elapsed);
	enum move {FORWARD, BACKWARD, LEFT, RIGHT};
	bool toleranceFilter(Record r);
	void restore();
	//
	vector<Record> data;
	float angle, lidarAngle;
	Vertex location;
	float xV, yV;
	float minVar, maxVar;
	float tolerance;
	Grid grid;
	set<Vertex> gridOverlay;
	set<Vertex> difference;
	float lookAhead;
	Vertex previousLocation;
	float previousAngle;
	bool stuck;
	set<Vertex> intersectCells;
	int strategy;
	double startGran, minGran, split;
	float turned;
	//Strategy 3 variables (for displaying)
	static const int quadW = 8, quadH = 8;
	int quadrant[quadW][quadH];
	double quadX, quadY;
	int curX, curY;
};

Behaviour::Behaviour(Robot *r, double startGran, double minGran, double split, int strategy) {
	this->r = r;
	this->angle = 0;
	this->lidarAngle = 0;
	location = Vertex(0, 0);
	xV = 0; yV = 0;
	minVar = 99999; maxVar = 0;
	tolerance = 99999;
	r->updateLidar();
	data.push_back(Record(location.x, location.y, angle, lidarAngle, r->lidarDistance, xV, yV));
	grid = Grid(startGran, minGran, split, &data);
	srand((unsigned)time(NULL));
	lookAhead = sqrt(r->width * r->width + r->height * r->height) -
		(r->width > r->height ? r->width : r->height);
	previousLocation.x = -1; previousLocation.y = -1;
	previousAngle = -1;
	stuck = false;
	this->strategy = strategy;
	for (int y = 0; y < quadH; y++)
		for (int x = 0; x < quadW; x++)
			quadrant[x][y] = 0;
	this->startGran = startGran;
	this->minGran = minGran;
	this->split = split;
	turned = 0;
}

//Resets behaviour to its original state for a new test.
void Behaviour::restore() {
	angle = 0;
	lidarAngle = 0;
	location = Vertex(0, 0);
	xV = 0; yV = 0;
	minVar = 99999; maxVar = 0;
	tolerance = 99999;
	r->updateLidar();
	data.clear();
	data.push_back(Record(location.x, location.y, angle, lidarAngle, r->lidarDistance, xV, yV));
	grid = Grid(startGran, minGran, split, &data);
	lookAhead = sqrt(r->width * r->width + r->height * r->height) -
		(r->width > r->height ? r->width : r->height);
	previousLocation.x = -1; previousLocation.y = -1;
	previousAngle = -1;
	stuck = false;
	for (int y = 0; y < quadH; y++)
		for (int x = 0; x < quadW; x++)
			quadrant[x][y] = 0;
	turned = 0;
}

void Behaviour::runStrategy(float elapsed) {

	switch(strategy) {
		case 1: {
			//Calculate the co-ordinates of the front right of robot.
			float y2 = r->width / 2;
			float x2 = r->height / 2;
			float radians = angle * (float)PI / 180;
			Vertex fr;
			fr.x = x2 * cos(radians) + y2 * sin(radians) + location.x;
			fr.y = x2 * sin(radians) - y2 * cos(radians) + location.y;

			//Calculate the equation of the line.
			float a = angle - 45;
			if (a < 0) a += 360;
			float m = tan(a * (float)PI / 180);
			float c = fr.y - m * fr.x;

			//Determine which way to step in x and y.
			int stepX = 0, stepY = 0;
			if (a > 0 && a < 180) stepY = 1;
			else if (a < 360) stepY = -1;
			if (a > 0 && a < 90 || a > 270 && a < 360) stepX = 1;
			else if (a > 90 && a < 270) stepX = -1;

			//Step in x, calculate the corresponding y, add to set.
			intersectCells.clear();
			bool cond = true;
			for (int x = grid.cellX(fr.x); stepX != 0 && cond; x += stepX) {
				int y = grid.cellY(m * grid.worldX(x) + c);
				intersectCells.insert(Vertex((float)x, (float)y));
				if (y == 0 || y == grid.height) break;
				cond = (stepX == 1) ? x <= grid.width : x >= 0;
			}

			//Step in y, calculate corresponding x, add to set.
			cond = true;
			for (int y = grid.cellY(fr.y); stepY != 0 && cond; y += stepY) {
				int x = grid.cellX((grid.worldY(y) - c) / m);
				intersectCells.insert(Vertex((float)x, (float)y));
				if (x == 0 || x == grid.width) break;
				cond = (stepY == 1) ? y <= grid.height : y >= 0;
			}

			//Verify cells lie in the correct quadrant.
			//This is needed due to errors when tan(n) -> +- infinity
			int startX = grid.cellX(fr.x);
			int startY = grid.cellY(fr.y);
			set<Vertex> errors;
			for (set<Vertex>::iterator i = intersectCells.begin(); i != intersectCells.end(); i++)
				if ((i->x - startX) * stepX < 0 || (i->y - startY) * stepY < 0)
					errors.insert(*i);
			set<Vertex> difference;
			set_difference(intersectCells.begin(), intersectCells.end(),
						   errors.begin(), errors.end(),
						   inserter(difference, difference.end()));
			intersectCells = difference;


			//Find the distance to nearest cell with p > 0, or to the cell at the edge of grid.
			float d = 999999;
			for (set<Vertex>::iterator i = intersectCells.begin(); i != intersectCells.end(); i++) {
				if (i->x < 1 || i->x >= grid.width - 1 || i->y < 1 || i->y >= grid.height - 1 ||
				grid.cells[(int)i->x][(int)i->y] > 0) {
					float x = (float)grid.worldX((int)i->x) + 0.5f * (float)grid.curGran;
					float y = (float)grid.worldY((int)i->y) + 0.5f * (float)grid.curGran;
					float distance = sqrt((fr.x - x) * (fr.x - x) + (fr.y - y) * (fr.y - y));
					if (distance < d) d = distance;
				}
			}

			//Remove all other cells from the set (display only).
			set<Vertex> tooFar;
			for (set<Vertex>::iterator i = intersectCells.begin(); i != intersectCells.end(); i++) {
				float x = (float)grid.worldX((int)i->x) + 0.5f * (float)grid.curGran;
				float y = (float)grid.worldY((int)i->y) + 0.5f * (float)grid.curGran;
				float distance = sqrt((fr.x - x) * (fr.x - x) + (fr.y - y) * (fr.y - y));
				if (distance > d) tooFar.insert(Vertex(i->x, i->y));
			}
			difference.clear();
			set_difference(intersectCells.begin(), intersectCells.end(),
						   tooFar.begin(), tooFar.end(),
						   inserter(difference, difference.end()));
			intersectCells = difference;

			//Try to maintain distance between 1 and 2 robot width's from the wall.
			float min = sqrt(2 * ((r->width * 1) * (r->width * 1)));
			float max = sqrt(2 * ((r->width * 2) * (r->width * 2)));

			//Cycle between forward and turning.
			static bool turn = false;
			turn = !turn;

			//Prevent left-right cycles. (forward: 0, left: 1, right: 2)
			static int lastMove = 1;

			//Turn away, if too close to the wall.
			if (d < min) {
				if (lastMove == 2) {
					forward(r->moveRate * elapsed);
					lastMove = 0;
				}
				else
					if (turn) {
						left(r->turnRate * elapsed);
						lastMove = 1;
					}
					else
						if (collision(FORWARD, r->moveRate * elapsed) == 0) {
							forward(r->moveRate * elapsed);
							lastMove = 0;
						}
						else {
							left(r->turnRate * elapsed);
							lastMove = 1;
						}
			}

			//Turn towards, if too far from the wall.
			else if (d > max) {
				if (lastMove == 1) {
					forward(r->moveRate * elapsed);
					lastMove = 0;
				}
				else
					if (turn) {
						right(r->turnRate * elapsed);
						lastMove = 2;
					}
					else
						if (collision(FORWARD, r->moveRate * elapsed) == 0) {
							forward(r->moveRate * elapsed);
							lastMove = 0;
						}
						else {
							right(r->turnRate * elapsed);
							lastMove = 2;
						}
			}

			//Otherwise, go straight on.
			else {
				forward(r->moveRate * elapsed);
				lastMove = 0;
			}

			break;
		}
		//Collision turning strategy.
		case 2: {
			static int direction = -1;
			static float remaining;
			float directionFactor = 2;
			static bool lastBack = false;

			//Try to move forward.
			if (collision(FORWARD, lookAhead) == 0 && !lastBack) {
				forward(r->moveRate * elapsed);
				//Decrement remaining by amount moved.
				remaining -= r->moveRate * elapsed;
				//If remaining expired, then set direction to null.
				if (remaining <= 0) direction = -1;
			}
			//Obstacle in front.
			else {
				//Set remaining to the longest out of width and height * a directionFactor.
				remaining = (r->width > r->height ? r->width : r->height) * directionFactor;
				//If remaining expired, choose a new direction.
				if (direction == -1) direction = rand() % 2;
				//Otherwise move in the same direction as previously.
				if (direction) right(r->turnRate * elapsed);
				else left(r->turnRate * elapsed);
				lastBack = false;
			}

			//If robot becomes stuck, as a last effort, attempt to move either direction.
			if (r->location == previousLocation && r->angle == previousAngle) {
				backward(r->moveRate * elapsed);
				lastBack = true;
			}
			if (r->location == previousLocation && r->angle == previousAngle) {
				if (direction) left(r->turnRate * elapsed);
				else right(r->turnRate * elapsed);
			}

			break;
		}		
		//Quadrants
		case 3: {
			//Increment current quadrant number of turns.
			curX = int(quadW * (float)grid.cellX(location.x) / grid.width);
			curY = int(quadH * (float)grid.cellY(location.y) / grid.height);
			quadrant[curX][curY]++;

			//Find the minimum turns for neighbours.
			int min = 999999;
			if (curX < quadW - 1 && quadrant[curX + 1][curY] < min)
				min = quadrant[curX + 1][curY];
			if (curY < quadH - 1 && quadrant[curX][curY + 1] < min)
				min = quadrant[curX][curY + 1];
			if (curX > 0 && quadrant[curX - 1][curY] < min)
				min = quadrant[curX - 1][curY];
			if (curY > 0 && quadrant[curX][curY - 1] < min)
				min = quadrant[curX][curY - 1];

			//Find the neighbours with the minimum turns.
			bool rN = false, tN = false, lN = false, bN = false;
			if (curX < quadW - 1 && quadrant[curX + 1][curY] == min)
				rN = true;
			if (curY < quadH - 1 && quadrant[curX][curY + 1] == min)
				tN = true;
			if (curX > 0 && quadrant[curX - 1][curY] == min)
				lN = true;
			if (curY > 0 && quadrant[curX][curY - 1] == min)
				bN = true;

			//Find the co-ordinates of nearest angular neighbour with minimum turns.
			//Calculate vector of robot's bearing.
			double rX = cos(angle * PI / 180);
			double rY = sin(angle * PI / 180);

			//Chooses the nearest angular cell as second ordering.
			double nearestAngle = 361;
			if (rN) {
				double xC = grid.worldX(int(((float)curX + 1 + 0.5f) * (float)grid.width / quadW));
				double yC = grid.worldY(int(((float)curY + 0.5f) * (float)grid.height / quadH));
				double vX = xC - location.x;
				double vY = yC - location.y;
				double mod = sqrt(vX * vX + vY * vY);
				vX /= mod;
				vY /= mod;
				double a = (atan2(rY, rX) - atan2(vY, vX)) / PI * 180;
				if (a < 0) a += 360;
				double turnAngle = (a < 360 - a) ? a : 360 - a;
				if (turnAngle < nearestAngle) {
					quadX = xC; quadY = yC;
					nearestAngle = turnAngle;
				}
			}
			if (tN) {
				double xC = grid.worldX(int(((float)curX + 0.5f) * (float)grid.width / quadW));
				double yC = grid.worldY(int(((float)curY + 1 + 0.5f) * (float)grid.height / quadH));
				double vX = xC - location.x;
				double vY = yC - location.y;
				double mod = sqrt(vX * vX + vY * vY);
				vX /= mod;
				vY /= mod;
				double a = (atan2(rY, rX) - atan2(vY, vX)) / PI * 180;
				if (a < 0) a += 360;
				double turnAngle = (a < 360 - a) ? a : 360 - a;
				if (turnAngle < nearestAngle) {
					quadX = xC; quadY = yC;
					nearestAngle = turnAngle;
				}
			}
			if (lN) {
				double xC = grid.worldX(int(((float)curX - 1 + 0.5f) * (float)grid.width / quadW));
				double yC = grid.worldY(int(((float)curY + 0.5f) * (float)grid.height / quadH));
				double vX = xC - location.x;
				double vY = yC - location.y;
				double mod = sqrt(vX * vX + vY * vY);
				vX /= mod;
				vY /= mod;
				double a = (atan2(rY, rX) - atan2(vY, vX)) / PI * 180;
				if (a < 0) a += 360;
				double turnAngle = (a < 360 - a) ? a : 360 - a;
				if (turnAngle < nearestAngle) {
					quadX = xC; quadY = yC;
					nearestAngle = turnAngle;
				}
			}
			if (bN) {
				double xC = grid.worldX(int(((float)curX + 0.5f) * (float)grid.width / quadW));
				double yC = grid.worldY(int(((float)curY - 1 + 0.5f) * (float)grid.height / quadH));
				double vX = xC - location.x;
				double vY = yC - location.y;
				double mod = sqrt(vX * vX + vY * vY);
				vX /= mod;
				vY /= mod;
				double a = (atan2(rY, rX) - atan2(vY, vX)) / PI * 180;
				if (a < 0) a += 360;
				double turnAngle = (a < 360 - a) ? a : 360 - a;
				if (turnAngle < nearestAngle) {
					quadX = xC; quadY = yC;
					nearestAngle = turnAngle;
				}
			}

			//Calculate vector from robot to center of quadrant.
			double vX = quadX - location.x;
			double vY = quadY - location.y;
			double mod = sqrt(vX * vX + vY * vY);
			vX /= mod;
			vY /= mod;

			//Calculate angle between the two vectors.
			double a = (atan2(rY, rX) - atan2(vY, vX)) / PI * 180;
			if (a < 0) a += 360;

			//Calculate preferred turn direction.
			bool turnLeft = true;
			if (a < 180) turnLeft = false;

			//Test whether current bearing is near enough to desired bearing.
			bool turn = true;
			double acceptable = 5 * elapsed * r->turnRate;
			if (a < acceptable / 2 || a > 360 - acceptable / 2)
				turn = false;

			//Uses strat 1's collision prevention, but with fixed ideas about direction.
			static int direction = -1;
			static float remaining;
			float directionFactor = 2;
			static bool lastBack = false;

			//Try to move forward.
			if (collision(FORWARD, lookAhead) == 0 && !lastBack) {
				if (remaining > 0) forward(r->moveRate * elapsed);
				else {
					if (turn)
						if (turnLeft) left(r->turnRate * elapsed);
						else right(r->turnRate * elapsed);
					else forward(r->moveRate * elapsed);
				}
				//Decrement remaining by amount moved.
				remaining -= r->moveRate * elapsed;
				//If remaining expired, then set direction to null.
				if (remaining <= 0) direction = -1;
			}
			//Obstacle in front.
			else {
				//Set remaining to the longest out of width and height * a directionFactor.
				remaining = (r->width > r->height ? r->width : r->height) * directionFactor;
				//If remaining expired, choose a new direction.
				if (direction == -1) direction = rand() % 2;
				//Otherwise move in the same direction as previously.
				if (direction) right(r->turnRate * elapsed);
				else left(r->turnRate * elapsed);
				lastBack = false;
			}

			//If robot becomes stuck, as a last effort, attempt to move either direction.
			if (r->location == previousLocation && r->angle == previousAngle) {
				backward(r->moveRate * elapsed);
				lastBack = true;
			}
			if (r->location == previousLocation && r->angle == previousAngle) {
				if (direction) left(r->turnRate * elapsed);
				else right(r->turnRate * elapsed);
			}

			break;
		}
		//Modified quadrants
		//Toggles forwards and turning
		//Moves to nearest rather than closest angle
		case 4: {
			//Increment current quadrant number of turns.
			curX = int(quadW * (float)grid.cellX(location.x) / grid.width);
			curY = int(quadH * (float)grid.cellY(location.y) / grid.height);
			quadrant[curX][curY]++;

			//Find the minimum turns for neighbours.
			int min = 999999;
			if (curX < quadW - 1 && quadrant[curX + 1][curY] < min)
				min = quadrant[curX + 1][curY];
			if (curY < quadH - 1 && quadrant[curX][curY + 1] < min)
				min = quadrant[curX][curY + 1];
			if (curX > 0 && quadrant[curX - 1][curY] < min)
				min = quadrant[curX - 1][curY];
			if (curY > 0 && quadrant[curX][curY - 1] < min)
				min = quadrant[curX][curY - 1];

			//Find the neighbours with the minimum turns.
			bool rN = false, tN = false, lN = false, bN = false;
			if (curX < quadW - 1 && quadrant[curX + 1][curY] == min)
				rN = true;
			if (curY < quadH - 1 && quadrant[curX][curY + 1] == min)
				tN = true;
			if (curX > 0 && quadrant[curX - 1][curY] == min)
				lN = true;
			if (curY > 0 && quadrant[curX][curY - 1] == min)
				bN = true;

			//Find the co-ordinates of nearest angular neighbour with minimum turns.
			//Calculate vector of robot's bearing.
			double rX = cos(angle * PI / 180);
			double rY = sin(angle * PI / 180);

			//Find the co-ordinates of nearest neighbour with minimum turns.
			double nearest = 999999;
			if (rN) {
				double xC = grid.worldX(int(((float)curX + 1 + 0.5f) * (float)grid.width / quadW));
				double yC = grid.worldY(int(((float)curY + 0.5f) * (float)grid.height / quadH));
				double distance = sqrt((xC - location.x) * (xC - location.x) + (yC - location.y) * (yC - location.y));
				if (distance < nearest) {
					nearest = distance;
					quadX = xC; quadY = yC;
				}
			}
			if (tN) {
				double xC = grid.worldX(int(((float)curX + 0.5f) * (float)grid.width / quadW));
				double yC = grid.worldY(int(((float)curY + 1 + 0.5f) * (float)grid.height / quadH));
				double distance = sqrt((xC - location.x) * (xC - location.x) + (yC - location.y) * (yC - location.y));
				if (distance < nearest) {
					nearest = distance;
					quadX = xC; quadY = yC;
				}
			}
			if (lN) {
				double xC = grid.worldX(int(((float)curX - 1 + 0.5f) * (float)grid.width / quadW));
				double yC = grid.worldY(int(((float)curY + 0.5f) * (float)grid.height / quadH));
				double distance = sqrt((xC - location.x) * (xC - location.x) + (yC - location.y) * (yC - location.y));
				if (distance < nearest) {
					nearest = distance;
					quadX = xC; quadY = yC;
				}
			}
			if (bN) {
				double xC = grid.worldX(int(((float)curX + 0.5f) * (float)grid.width / quadW));
				double yC = grid.worldY(int(((float)curY - 1 + 0.5f) * (float)grid.height / quadH));
				double distance = sqrt((xC - location.x) * (xC - location.x) + (yC - location.y) * (yC - location.y));
				if (distance < nearest) {
					nearest = distance;
					quadX = xC; quadY = yC;
				}
			}

			//Calculate vector from robot to center of quadrant.
			double vX = quadX - location.x;
			double vY = quadY - location.y;
			double mod = sqrt(vX * vX + vY * vY);
			vX /= mod;
			vY /= mod;

			//Calculate angle between the two vectors.
			double a = (atan2(rY, rX) - atan2(vY, vX)) / PI * 180;
			if (a < 0) a += 360;

			//Calculate preferred turn direction.
			bool turnLeft = true;
			if (a < 180) turnLeft = false;

			//Test whether current bearing is near enough to desired bearing.
			bool turn = true;
			double acceptable = 5 * elapsed * r->turnRate;
			if (a < acceptable / 2 || a > 360 - acceptable / 2)
				turn = false;

			//Uses strat 1's collision prevention, but with fixed ideas about direction.
			static int direction = -1;
			static float remaining;
			float directionFactor = 2;
			static bool lastBack = false;
			static bool toggle = false;
			toggle = !toggle;

			//Try to move forward.
			if (collision(FORWARD, lookAhead) == 0 && !lastBack) {
				if (remaining > 0) forward(r->moveRate * elapsed);
				else {
					if (turn)
						if (toggle) forward(r->moveRate * elapsed);
						else
							if (turnLeft) left(r->turnRate * elapsed);
							else right(r->turnRate * elapsed);
					else forward(r->moveRate * elapsed);
				}
				//Decrement remaining by amount moved.
				remaining -= r->moveRate * elapsed;
				//If remaining expired, then set direction to null.
				if (remaining <= 0) direction = -1;
			}
			//Obstacle in front.
			else {
				//Set remaining to the longest out of width and height * a directionFactor.
				remaining = (r->width > r->height ? r->width : r->height) * directionFactor;
				//If remaining expired, choose a new direction.
				if (direction == -1) direction = rand() % 2;
				//Otherwise move in the same direction as previously.
				if (direction) right(r->turnRate * elapsed);
				else left(r->turnRate * elapsed);
				lastBack = false;
			}

			//If robot becomes stuck, as a last effort, attempt to move either direction.
			if (r->location == previousLocation && r->angle == previousAngle) {
				backward(r->moveRate * elapsed);
				lastBack = true;
			}
			if (r->location == previousLocation && r->angle == previousAngle) {
				if (direction) left(r->turnRate * elapsed);
				else right(r->turnRate * elapsed);
			}


			break;
		}
		
	}

}

//Must move: moveRate * elapsed, or turn: turnRate * elapsed.
void Behaviour::nextMove(float elapsed) {
	//Collect some LIDAR data before starting.
	if (turned < 360) {
		right(r->turnRate * elapsed);
		turned += r->turnRate * elapsed;
		return;
	}

	runStrategy(elapsed);

	//Force early exit if robot becomes stuck.
	if (r->location == previousLocation && r->angle == previousAngle)
		stuck = true;
	previousLocation = r->location;
	previousAngle = r->angle;

	//Reset angle in case of unexpected collision.
	angle = r->angle;

	//Update the robot's position on the grid.
	gridOverlay = overlay(location, angle);
}

//Must update lidar: lidarRate * elapsed.
void Behaviour::nextLidar(float elapsed) {
	lidar(r->lidarRate * elapsed);

	if (r->lidarBeacon()) {
		//Use the beacon to set the exact robot position and rotation.
		location.x = r->location.x - r->startLocation.x;
		location.y = r->location.y - r->startLocation.y;
		angle = r->angle;
		lidarAngle = r->lidarAngle;

		//Reset the x, y variance.
		xV = 0;
		yV = 0;
	}

	//Add the record and update the grid.
	data.push_back(Record(location.x, location.y, angle, lidarAngle, r->lidarDistance, xV, yV));

	//Calculates the obstacle vertex and maps it onto grid.
	Vertex v = getVertex(location.x, location.y, lidarAngle, r->lidarDistance);
	grid.mapPoint(v.x, v.y, xV, yV);

	//Update the minimum and maximum variances.
	if (xV + yV < minVar) minVar = xV + yV;
	if (xV + yV > maxVar) maxVar = xV + yV;

	if (r->lidarBeacon()) {
		//Kalman filter update.
		//Iterate backwards over the data until a record with 0 variance.
		vector<Record> reverse;
		reverse.push_back(data.back());
		//grid.revert();
		for (int i = data.size() - 2; !(data[i].xV == 0 || data[i].yV == 0); i--) {
			//Find the distance travelled.
			float dx = data[i].x - data[i + 1].x;
			float dy = data[i].y - data[i + 1].y;
			float distance = sqrt(dx * dx + dy * dy);

			//Find the reverse expected location.
			float radians = data[i - 1].b * (float)PI / 180;
			float x = reverse.back().x - distance * cos(radians);
			float y = reverse.back().y - distance * sin(radians);

			//Find the reverse variances.
			float rxV = reverse.back().xV + abs(dx) * r->mNoise;
			float ryV = reverse.back().yV + abs(dy) * r->mNoise;

			//Add the record (-1 are unused).
			reverse.push_back(Record(x, y, -1, -1, -1, rxV, ryV));
		}

		//Calculate best estimate locations based on variance weightings.
		for (unsigned int r = 1, d = data.size() - 2; r < reverse.size(); r++, d--) {
			//Mean location.
			float x = data[d].x + (data[d].xV / (data[d].xV + reverse[r].xV)) * (reverse[r].x - data[d].x);
			float y = data[d].y + (data[d].yV / (data[d].yV + reverse[r].yV)) * (reverse[r].y - data[d].y);

			//Improved variance.
			float xVar = data[d].xV * reverse[r].xV / (data[d].xV + reverse[r].xV);
			float yVar = data[d].yV * reverse[r].yV / (data[d].yV + reverse[r].yV);

			//Update the data records.
			data[d].x = x;
			data[d].y = y;
			data[d].xV = xVar;
			data[d].yV = yVar;

			//Map the improved points to grid.
			Vertex v = getVertex(data[d].x, data[d].y, data[d].l, data[d].d);
			grid.mapPoint(v.x, v.y, data[d].xV, data[d].yV, true);
		}
		reverse.clear();
	}
}

//Move the robot forward, then update the expected location.
void Behaviour::forward(float amount) {
	r->forward(amount);
	float radians = angle * (float)PI / 180;
	float dx = amount * cos(radians);
	float dy = amount * sin(radians);
	location.x += dx;
	location.y += dy;
	xV += abs(dx) * r->mNoise;
	yV += abs(dy) * r->mNoise;
}

//Move the robot backward, then update the expected location.
void Behaviour::backward(float amount) {
	r->backward(amount);
	float radians = angle * (float)PI / 180;
	float dx = amount * cos(radians);
	float dy = amount * sin(radians);
	location.x -= dx;
	location.y -= dy;
	xV += abs(dx) * r->mNoise;
	yV += abs(dy) * r->mNoise;
}

//Turn the robot left, then update the expected rotation.
void Behaviour::left(float angle) {
	r->left(angle);
	this->angle += angle;
	if (this->angle >= 360) this->angle -= 360;
}

//Turn the robot right, then update the expected rotation.
void Behaviour::right(float angle) {
	r->right(angle);
	this->angle -= angle;
	if (this->angle < 0) this->angle += 360;
}

//Turn the lidar, then update the expected lidar rotation.
void Behaviour::lidar(float angle) {
	r->lidar(angle);
	lidarAngle += angle;
	if (lidarAngle >= 360) lidarAngle -= 360;
}

//Calculates the vertex at a lidar intercept.
Vertex Behaviour::getVertex(float x, float y, float l, float d) {
	float radians = l * (float)PI / 180;
	float vX = x + d * cos(radians);
	float vY = y + d * sin(radians);
	return Vertex(vX, vY);
}

//Deprecated
bool Behaviour::toleranceFilter(Record r) {
	if (r.xV + r.yV <= tolerance) return true;
	else return false;
}

set<Vertex> Behaviour::overlay(Vertex v, float a) {
	//Calculate the corner positions of robot.
	float y2 = r->width / 2;
	float x2 = r->height / 2;
	float radians = a * (float)PI / 180;
	Vertex frontRight, frontLeft, backLeft, backRight;
	frontRight.x = x2 * cos(radians) + y2 * sin(radians) + v.x;
	frontRight.y = x2 * sin(radians) - y2 * cos(radians) + v.y;
	frontLeft.x = x2 * cos(radians) - y2 * sin(radians) + v.x;
	frontLeft.y = x2 * sin(radians) + y2 * cos(radians) + v.y;
	backLeft.x = -x2 * cos(radians) - y2 * sin(radians) + v.x;
	backLeft.y = -x2 * sin(radians) + y2 * cos(radians) + v.y;
	backRight.x = -x2 * cos(radians) + y2 * sin(radians) + v.x;
	backRight.y = -x2 * sin(radians) - y2 * cos(radians) + v.y;

	//Construct a polygon from the robot corners.
	Polygon p;
	p.addVertex(frontRight);
	p.addVertex(frontLeft);
	p.addVertex(backLeft);
	p.addVertex(backRight);

	//Find the minimum and maximum cells.
	float minX = frontRight.x;
	if (frontLeft.x < minX) minX = frontLeft.x;
	if (backLeft.x < minX) minX = backLeft.x;
	if (backRight.x < minX) minX = backRight.x;
	int xMin = grid.cellX(minX);

	float minY = frontRight.y;
	if (frontLeft.y < minY) minY = frontLeft.y;
	if (backLeft.y < minY) minY = backLeft.y;
	if (backRight.y < minY) minY = backRight.y;
	int yMin = grid.cellY(minY);

	float maxX = frontRight.x;
	if (frontLeft.x > maxX) maxX = frontLeft.x;
	if (backLeft.x > maxX) maxX = backLeft.x;
	if (backRight.x > maxX) maxX = backRight.x;
	int xMax = grid.cellX(maxX);

	float maxY = frontRight.y;
	if (frontLeft.y > maxY) maxY = frontLeft.y;
	if (backLeft.y > maxY) maxY = backLeft.y;
	if (backRight.y > maxY) maxY = backRight.y;
	int yMax = grid.cellY(maxY);

	//Find the cells of the grid inside the robot polygon.
	set<Vertex> ret;
	for (int y = yMin; y <= yMax; y++)
		for (int x = xMin; x <= xMax; x++)
			if (p.inside(Vertex((float)grid.worldX(x), (float)grid.worldY(y)))) {
				ret.insert(Vertex((float)x, (float)y));
				ret.insert(Vertex((float)x - 1, (float)y));
				ret.insert(Vertex((float)x - 1, (float)y - 1));
				ret.insert(Vertex((float)x, (float)y - 1));
			}

	//Find the cells of the grid containing the robot corners.
	for (list<Vertex>::iterator i = p.vertices.begin(); i != p.vertices.end(); i++)
		ret.insert(Vertex((float)grid.cellX(i->x) - 1, float(grid.cellY(i->y) - 1)));


	return ret;
}

//Returns the probability of a collision from the given move.
float Behaviour::collision(int move, float amount) {
	//Calculate the expected location and angle of the robot.
	float radians = angle * (float)PI / 180;
	float x, y, a;
	switch (move) {
		case FORWARD:
			x = location.x + amount * cos(radians);
			y = location.y + amount * sin(radians);
			a = angle;
			break;
		case BACKWARD:
			x = location.x - amount * cos(radians);
			y = location.y - amount * sin(radians);
			a = angle;
			break;
		case LEFT:
			x = location.x;
			y = location.y;
			a = angle + amount;
			if (a >= 360) a -= 360;
			break;
		case RIGHT:
			x = location.x;
			y = location.y;
			a = angle - amount;
			if (a <= 0) a += 360;
			break;
	}

	//Find the additional vertices the robot would overlap by set-difference.
	set<Vertex> futureOverlap = overlay(Vertex(x, y), a);
	difference.clear();
	set_difference(futureOverlap.begin(), futureOverlap.end(),
				   gridOverlay.begin(), gridOverlay.end(),
				   inserter(difference, difference.end()));

	//Sum the probability of collision over the area.
	float p = 0;
	for (set<Vertex>::iterator i = difference.begin(); i != difference.end(); i++) {
		if (i->x >= 0 && i->x < grid.width && i->y >= 0 && i->y < grid.height) {
			p += (float)grid.cells[(int)i->x][(int)i->y];
			if (p >= 1) return 1;
		}
	}

	return p;
}

#endif