#ifndef GRID_H
#define GRID_H

#include <vector>
#include <boost/math/distributions/normal.hpp>
#include "Record.h"
using namespace std;
using namespace boost::math;

class Cell {
public:
	Cell(int x, int y, double p) : x(x), y(y), p(p) {}
	int x, y;
	double p;
};

class Grid {
public:
	Grid() {}
	Grid(double startGran, double minGran, double splitDeterminant, vector<Record> *data);
	void mapPoint(double x, double y, double xV, double yV, bool kalman);
	Vertex getVertex(float x, float y, float l, float d);
	void remap();
	int cellX(double worldX);
	int cellY(double worldY);
	double worldX(int cellX);
	double worldY(int cellY);
	void divide();
	double completeness();
	double accuracy();
	void revert();
	//
	double curGran, minGran;
	vector<Record> *data;  
	double xMin, xMax, yMin, yMax; //Points
	double xFrom, xTo, yFrom, yTo; //Grid
	int width, height;
	vector<vector<double>> cells;
	list<Cell> changedCells;
	bool firstVector;
	double splitDeterminant;
};

Grid::Grid(double startGran, double minGran, double splitDeterminant, vector<Record> *data) {
	this->curGran = startGran;
	this->minGran = minGran;
	this->splitDeterminant = splitDeterminant;
	this->data = data;
	xMin = 0; xMax = 0;
	yMin = 0; yMax = 0;
	xFrom = 0; xTo = 0;
	yFrom = 0; yTo = 0;
	width = 0; height = 0;
	firstVector = true;
}

//Maps the given point onto the grid.
void Grid::mapPoint(double x, double y, double xV, double yV, bool kalman = false) {
	//Standard deviation.
	double xSD = sqrt(xV);
	double ySD = sqrt(yV);

	//Calculate 3sd ellipse bounding box.
	double xVMin = x - 3 * xSD;
	double xVMax = x + 3 * xSD;
	double yVMin = y - 3 * ySD;
	double yVMax = y + 3 * ySD;

	//Set mins/maxes.
	if (xVMin < xMin) xMin = xVMin;
	else if (xVMax > xMax) xMax = xVMax;
	if (yVMin < yMin) yMin = yVMin;
	else if (yVMax > yMax) yMax = yVMax;

	//Check if we need to resize.
	if (xMin < xFrom || xMax > xTo || yMin < yFrom || yMax > yTo || firstVector) {
		firstVector = false;

		//Calculate from/to.
		xFrom = floor(xMin / curGran) * curGran;
		xTo = ceil(xMax / curGran) * curGran;
		yFrom = floor(yMin / curGran) * curGran;
		yTo = ceil(yMax / curGran) * curGran;

		//Calculate width and height.
		width = abs((int)ceil(xMax / curGran)) + abs((int)floor(xMin / curGran)) + 1;
		height = abs((int)ceil(yMax / curGran)) + abs((int)floor(yMin / curGran)) + 1;

		//Resize the array.
		cells.resize(width);
		for (int i = 0; i < width; i++)
			cells[i].resize(height);

		//Remap all points and return.
		remap();
		return;
	}

	//Calculate cell min/max coords.
	int xCMin = cellX(xVMin);
	int xCMax = cellX(xVMax);
	int yCMin = cellY(yVMin);
	int yCMax = cellY(yVMax);

	//Loop through cells of bounding box.
	for (int yC = yCMin; yC <= yCMax; yC++)
		for (int xC = xCMin; xC <= xCMax; xC++) {
			//Calculate probability of current cell.
			double pX, pY;
			if (xSD == 0) pX = 1;
			else {
				normal_distribution<double> xNormal(x, xSD);
				pX = cdf(xNormal, worldX(xC + 1)) - cdf(xNormal, worldX(xC));
			}
			if (ySD == 0) pY = 1;
			else {
				normal_distribution<double> yNormal(y, ySD);
				pY = cdf(yNormal, worldY(yC + 1)) - cdf(yNormal, worldY(yC));
			}
			double p = pX * pY;

			//Combine probability independently. DEPRECATED
			//Problem: Causes 'over exposure' when multipass measurements.
			//if (addition) cells[xC][yC] = 1 - (1 - cells[xC][yC]) * (1 - p);
			//else  cells[xC][yC] = (cells[xC][yC] - p) / (1 - p);

			//Store the previous cell value in changedCells.
			//Pushing to the front means we don't need to check for containment.
			//Reducing complexity to n rather than n^2 when reverting.
			if (!kalman) changedCells.push_front(Cell(xC, yC, cells[xC][yC]));

			//Set probability to maximum of current cell or calculated probability.
			cells[xC][yC] = (cells[xC][yC] > p) ? cells[xC][yC] : p;

			//Split granularity if adjacent cells have probability > determinant.
			if (p > splitDeterminant) {
				//Test surrounding cells are contained within grid.
				bool left = false, right = false, bottom = false, top = false;
				if (xC - 1 >= 0) left = true;
				if (xC + 1 < width) right = true;
				if (yC - 1 >= 0) bottom = true;
				if (yC + 1 < height) top = true;

				//Orthogonal cells.
				if (left && cells[xC - 1][yC] > splitDeterminant) { divide(); return; }
				if (right && cells[xC + 1][yC] > splitDeterminant) { divide(); return; }
				if (bottom && cells[xC][yC - 1] > splitDeterminant) { divide(); return; }
				if (top && cells[xC][yC + 1] > splitDeterminant) { divide(); return; }

				//Diagonal cells.
				if (left && bottom && cells[xC - 1][yC - 1] > splitDeterminant) { divide(); return; }
				if (left && top && cells[xC - 1][yC + 1] > splitDeterminant) { divide(); return; }
				if (right && bottom && cells[xC + 1][yC - 1] > splitDeterminant) { divide(); return; }
				if (right && top && cells[xC + 1][yC + 1] > splitDeterminant) { divide(); return; }
			}
		}
}

//Clears the array and remaps all points.
void Grid::remap() {
	//Clear.
	for (int y = 0; y < height; y++)
		for (int x = 0; x < width; x++)
			cells[x][y] = 0;
	changedCells.clear();
	
	//Iterate through data and map points.
	for (vector<Record>::iterator i = data->begin(); i != data->end(); i++) {
		Vertex v = getVertex(i->x, i->y, i->l, i->d);
		mapPoint(v.x, v.y, i->xV, i->yV);
	}
}

//Calculates the x cell coord of a world coord.
int Grid::cellX(double worldX) {
	int x = 0;
	for (double i = xFrom; i <= xTo; i += curGran, x++)
		if (i >= worldX) return x;
	return x;
}

//Calculates the y cell coord of a world coord.
int Grid::cellY(double worldY) {
	int y = 0;
	for (double i = yFrom; i <= yTo; i += curGran, y++)
		if (i >= worldY) return y;
	return y;
}

//Calculates the x world coord of a cell coord.
double Grid::worldX(int cellX) {
	return xFrom + curGran * cellX;
}

//Calculates the y world coord of a cell coord.
double Grid::worldY(int cellY) {
	return yFrom + curGran * cellY;
}

//Calculates the vertex at a lidar intercept.
Vertex Grid::getVertex(float x, float y, float l, float d) {
	float radians = l * (float)PI / 180;
	float vX = x + d * cos(radians);
	float vY = y + d * sin(radians);
	return Vertex(vX, vY);
}

//Divides the granularity in two and remaps points.
void Grid::divide() {
	//Limit granuliarty to minimum.
	if (curGran == minGran) return;
	curGran /= 2;
	if (curGran < minGran) curGran = minGran;

	//Reset the vector.
	cells.clear();
	firstVector = true;
	changedCells.clear();
	
	//Iterate through data and map points.
	for (vector<Record>::iterator i = data->begin(); i != data->end(); i++) {
		Vertex v = getVertex(i->x, i->y, i->l, i->d);
		mapPoint(v.x, v.y, i->xV, i->yV);
	}
}

//Calculates the ratio of non-zero grid cells.
double Grid::completeness() {
	int n = 0;
	for (int y = 0; y < height; y++)
		for (int x = 0; x < width; x++)
			if (cells[x][y] != 0)
				n++;
	return float(n) / width / height;
}

//Calculates the average variance of all non-zero cells.
double Grid::accuracy() {
	int n = 0;
	double v = 0;
	for (int y = 0; y < height; y++)
		for (int x = 0; x < width; x++)
			if (cells[x][y] != 0) {
				n++;
				v += cells[x][y];
			}
	return 1 - v / double(n);
}

//Revert the grid to the previous kalman.
void Grid::revert() {
	while (!changedCells.empty()) {
		cells[changedCells.front().x][changedCells.front().y] = changedCells.front().p;
		changedCells.pop_front();
	}
}

#endif