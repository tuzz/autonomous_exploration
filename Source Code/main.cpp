#include "Vertex.h"
#include "Polygon.h"
#include "Environment.h"
#include "Robot.h"
#include "Behaviour.h"
#include "Display.h"
#include "xml/pugixml.cpp"
using namespace pugi;
#include <iostream>
using namespace std;

int main(int argc, char **argv) {
	//If no file specified, assume 'e1s1.xml'.
	if (argc <= 1)
		argv[1] = "e1s1.xml";

	//Load the xml file, check if not found.
	xml_document xml;
	xml_parse_result result = xml.load_file(argv[1]);
	if (!result) {
		if (argc <= 1)
			cout << "No configuration file specified." << endl;
		else
			cout << "File cannot be found." << endl;
		printf("Press enter to exit...");
		cin.get();
		return 0;
	}

	//Get the number of tests from xml.
	int tests = atoi(xml.child("root").child_value("tests"));

	//Set up environment from xml.
	xml_node env = xml.child("root").child("environment");
	Environment e(env.attribute("width").as_float(), env.attribute("height").as_float());
	for (xml_node polygon = env.child("polygon"); polygon; polygon = polygon.next_sibling("polygon")) {
		Polygon p;
		for (xml_node vertex = polygon.child("vertex"); vertex; vertex = vertex.next_sibling("vertex")) {
			p.addVertex(Vertex(vertex.attribute("x").as_float(), vertex.attribute("y").as_float()));
		}
		if (polygon.attribute("beacon").as_bool()) e.addBeacon(p);
		else e.addObstacle(p);
	}

	//Set up robot from xml.
	xml_node robot = xml.child("root").child("robot");
	float robotWidth = robot.attribute("width").as_float();
	float robotHeight = robot.attribute("height").as_float();
	xml_node startVertex = robot.child("start").child("vertex");
	Vertex robotStart(startVertex.attribute("x").as_float(), startVertex.attribute("y").as_float());
	float moveRate = atof(robot.child_value("moveRate"));
	float turnRate = atof(robot.child_value("turnRate"));
	float lidarRate = atof(robot.child_value("lidarRate"));
	float noise = atof(robot.child_value("noise"));
	Robot r(robotWidth, robotHeight, robotStart, moveRate, turnRate, lidarRate, noise, 0, 0, &e);

	//Set up behaviour from xml.
	xml_node behaviour = xml.child("root").child("behaviour");
	xml_node grid = behaviour.child("grid");
	float startGran = atof(grid.child_value("startGran"));
	float minGran = atof(grid.child_value("minGran"));
	float split = atof(grid.child_value("split"));
	int strategy = atoi(behaviour.child_value("strategy"));
	Behaviour b(&r, startGran, minGran, split, strategy);

	//Set up display from xml.
	xml_node displayN = xml.child("root").child("display");
	int displayWidth = displayN.attribute("width").as_int();
	int displayHeight = displayN.attribute("height").as_int();
	if (displayWidth == 0 || displayHeight == 0) {
		printf("Invalid configuration file.\n");
		printf("Press enter to exit...");
		cin.get();
		return 0;
	}

	//Finish setting up display.
	xml_node defaultN = displayN.child("default");
	bool robotV = defaultN.child("robotV");
	bool lidarV = defaultN.child("lidarV");
	bool obstaclesV = defaultN.child("obstaclesV");
	bool beaconsV = defaultN.child("beaconsV");
	bool overlayV = defaultN.child("overlayV");
	bool collisionV = defaultN.child("collisionV");
	bool detectorV = defaultN.child("detectorV");
	bool ellipsesV = defaultN.child("ellipsesV");
	bool pathV = defaultN.child("pathV");
	bool gridV = defaultN.child("gridV");
	bool mappingsV = defaultN.child("mappingsV");
	bool verticesV = defaultN.child("verticesV");
	bool trackerV = defaultN.child("trackerV");
	bool quadV = defaultN.child("quadrantsV");
	bool seekV = defaultN.child("seekV");
	bool debugV = defaultN.child("debugV");
	float runtime = atof(displayN.child_value("runtime"));
	display(argc, argv, &e, &r, &b, displayWidth, displayHeight, robotV, lidarV, obstaclesV, beaconsV, overlayV,
			collisionV, detectorV, ellipsesV, pathV, gridV, mappingsV, verticesV, trackerV,
			quadV, seekV, debugV, runtime, tests);

	return 0;
}