#ifndef POLYGON_H
#define POLYGON_H

#include <list>

class Polygon {
public:
	Polygon() {}
	Polygon(std::list<Vertex> vertices);
	void addVertex(Vertex v);
	std::list<Vertex> vertices;
	bool sameSide(Vertex a, Vertex b, Vertex r, Vertex p);
	bool inside(Vertex v);
	bool overlaps(Polygon other);
};

Polygon::Polygon(std::list<Vertex> vertices) {
	this->vertices = vertices;
}

void Polygon::addVertex(Vertex v) {
	vertices.push_back(v);
}

//Checks if points r and p lie on the same side of the line connecting a and b.
bool Polygon::sameSide(Vertex a, Vertex b, Vertex r, Vertex p) {
	float m = (b.y - a.y) / (b.x - a.x);
	float eqR = m * (r.x - a.x) - (r.y - a.y);
	float eqP = m * (p.x - a.x) - (p.y - a.y);
	return ((eqR < 0 && eqP < 0) || (eqR > 0 && eqP > 0));
}

//Checks if Vertex v is inside this polygon.
bool Polygon::inside(Vertex v) {
	//Calculate the centre point.
	float centreX = 0, centreY = 0;

	for (std::list<Vertex>::iterator i = vertices.begin(); i != vertices.end(); i++) {
		centreX += i->x;
		centreY += i->y;
	}

	centreX /= vertices.size();
	centreY /= vertices.size();

	Vertex centre(centreX, centreY);

	//If the point does not lie on the same side of any of the connecting lines, return false.
	for (std::list<Vertex>::iterator i = vertices.begin(); i != --vertices.end(); )
		if (!sameSide(*i, *(i++), centre, v)) return false;
	if (!sameSide(vertices.front(), vertices.back(), centre, v)) return false;

	//Otherwise return true.
	return true;
}

//Checks if this polygon overlaps another polygon.
bool Polygon::overlaps(Polygon other) {
	//Check if any vertex of this polygon is in the other one.
	for (std::list<Vertex>::iterator i = vertices.begin(); i != vertices.end(); i++)
		if (other.inside(*i)) return true;

	//Check if any vertex of the other polygon is in this one.
	for (std::list<Vertex>::iterator i = other.vertices.begin(); i != other.vertices.end(); i++)
		if (inside(*i)) return true;

	//Otherwise return false.
	return false;
}

#endif