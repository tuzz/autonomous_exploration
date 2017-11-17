#ifndef DISPLAY_H
#define DISPLAY_H

#include <GL/glut.h>
#include "Environment.h"
#include "Robot.h"
#include "Behaviour.h"
#include "Record.h"
#include <fstream>

void display(int argc, char **argv, Environment *e, Robot *r);
void displayCallback();
void graphDisplay();
void timerCallback(int n);
void graphTimer(int n);
void keyboardCallback(unsigned char, int, int);
void graphReshape(int, int);
void reshapeCallback(int, int);
void menuStatusCallback(int);
void menuCallback(int);
void setMenuStrings();

double runtime = 0;
list<pair<float, float>> graphData;
int graphIndex = 0;
int mainWindow;
int graphWindow;
int windowWidth, windowHeight;

enum MENU_TYPE {
	MENU_SPACER0 = 1,
	VIEW_ROBOT,
	VIEW_LIDAR,
	VIEW_OBSTACLES,
	VIEW_BEACONS,
	MENU_SPACER1,
	VIEW_OVERLAY,
	VIEW_COLLISION,
	VIEW_DETECTOR,
	VIEW_ELLIPSES,
	VIEW_PATH,
	MENU_SPACER2,
	VIEW_GRID,
	VIEW_MAPPINGS,
	VIEW_VERTICES,
	MENU_SPACER3,
	VIEW_TRACKER,
	VIEW_QUADRANTS,
	VIEW_SEEK,
	MENU_SPACER4,
	VIEW_DEBUG,
	MENU_PAUSE,
	MENU_EXIT
};
int MENU;
bool robotE, lidarE, obstaclesE, beaconsE, overlayE, collisionE, detectorE,
	 ellipsesE, pathE, gridE, mappingsE, verticesE, trackerE, quadE, seekE, debugE,
	 PAUSED, MENU_OPEN;

Environment *DisplayE;
Robot *DisplayR;
Behaviour *DisplayB;
int xFrom, yFrom, xTo, yTo;
int lastTime = glutGet(GLUT_ELAPSED_TIME);
float maxTime;
bool timeLimit;
int maxTests;
string filename;
//Timeout value between redisplays before the animation is paused in milliseconds.
//May need to be set higher on older systems.
int pauseTimeout = 50;

void nextTest() {
	static int curTest = 1;
	curTest++;
	if (curTest > maxTests) exit(0);
	cout << "Test " << curTest << "/" << maxTests << endl;
	DisplayR->restore();
	DisplayB->restore();
	xFrom = 0; yFrom = 0; xTo = 0; yTo = 0;
	lastTime = glutGet(GLUT_ELAPSED_TIME);
	runtime = 0;
	graphData.clear();
	graphIndex = 0;
	glutPostWindowRedisplay(mainWindow);
	glutPostWindowRedisplay(graphWindow);
}

void display(int argc, char **argv, Environment *e, Robot *r, Behaviour *b, int width, int height,
			 bool robotV, bool lidarV, bool obstaclesV, bool beaconsV, bool overlayV, bool collisionV, bool detectorV,
			 bool ellipsesV, bool pathV, bool gridV, bool mappingsV, bool verticesV, bool trackerV, bool quadV,
			 bool seekV, bool debugV, float runtime, int tests) {
	DisplayE = e;
	DisplayR = r;
	DisplayB = b;
	maxTests = tests;
	filename = argv[1];

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(width, height);
	mainWindow = glutCreateWindow("Simulated Autonomous Exploration | Christopher Patuzzo");

	glClearColor(0.5, 0.5, 0.5, 1);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-0.025f * DisplayE->width, 1.025f * DisplayE->width, -0.025f * DisplayE->height, 1.025f * DisplayE->height, 1, -1);
	glMatrixMode(GL_MODELVIEW);

	//Turns per second, should be representative of the robot.
	//Atleast 60 is recommended for a smooth animation.
	int n = 100;
	glutDisplayFunc(displayCallback);
	glutTimerFunc(1000 / n, timerCallback, n);
	glutKeyboardFunc(keyboardCallback);
	glutReshapeFunc(reshapeCallback);
	glutMenuStateFunc(menuStatusCallback);

	//Set the blend function.
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//Set the default views.
	robotE = robotV; lidarE = lidarV;
	obstaclesE = obstaclesV; beaconsE = beaconsV;
	overlayE = overlayV; collisionE = collisionV;
	detectorE = detectorV;
	ellipsesE = ellipsesV; pathE = pathV;
	gridE = gridV; mappingsE = mappingsV;
	verticesE = verticesV; trackerE = trackerV;
	quadE = quadV, seekE = seekV;
	debugE = debugV;

	robotE = robotV; lidarE = lidarV; obstaclesE = obstaclesV;
	beaconsE = beaconsV; overlayE = overlayV; detectorE = detectorV;
	ellipsesE = ellipsesV; pathE = pathV; gridE = gridV;
	mappingsE = mappingsV; verticesE = verticesV; debugE = debugV;

	//Set the runtime.
	maxTime = runtime;
	if (runtime != 0) timeLimit = true;
	else timeLimit = false;

	//Build  and attach the menu.
	MENU = glutCreateMenu(menuCallback);
	glutAddMenuEntry("- - - Real - - -", MENU_SPACER0);
	glutAddMenuEntry("", VIEW_ROBOT);
	glutAddMenuEntry("", VIEW_LIDAR);
	glutAddMenuEntry("", VIEW_OBSTACLES);
	glutAddMenuEntry("", VIEW_BEACONS);
	glutAddMenuEntry("- - - Internalised - - -", MENU_SPACER1);
	glutAddMenuEntry("", VIEW_OVERLAY);
	glutAddMenuEntry("", VIEW_COLLISION);
	glutAddMenuEntry("", VIEW_DETECTOR);
	glutAddMenuEntry("", VIEW_ELLIPSES);
	glutAddMenuEntry("", VIEW_PATH);
	glutAddMenuEntry("- - - Data - - -", MENU_SPACER2);
	glutAddMenuEntry("", VIEW_GRID);
	glutAddMenuEntry("", VIEW_MAPPINGS);
	glutAddMenuEntry("", VIEW_VERTICES);
	glutAddMenuEntry("- - - Strategies - - -", MENU_SPACER3);
	glutAddMenuEntry("", VIEW_TRACKER);
	glutAddMenuEntry("", VIEW_QUADRANTS);
	glutAddMenuEntry("", VIEW_SEEK);
	glutAddMenuEntry("- - - Misc - - -", MENU_SPACER4);
	glutAddMenuEntry("", VIEW_DEBUG);
	glutAddMenuEntry("", MENU_PAUSE);
	glutAddMenuEntry("", MENU_EXIT);

	setMenuStrings();
	glutAttachMenu(GLUT_RIGHT_BUTTON);

	//Create the graph window.
	glutInitWindowSize(450, 450);
	graphWindow = glutCreateWindow("Graph Window");
	glutPositionWindow(width + 40, (height - 450) / 2);
	glutDisplayFunc(graphDisplay);
	glutReshapeFunc(graphReshape);
	glutKeyboardFunc(keyboardCallback);
	glutTimerFunc(1000 / n, graphTimer, n);
	windowWidth = width;
	windowHeight = height;

	cout << "Test 1/" << maxTests << endl;
	glutMainLoop();
}

void graphDisplay() {
	glClear(GL_COLOR_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-0.2, 1.1, -0.25, 1.2, 1, -1);
	glMatrixMode(GL_MODELVIEW);

	//Set the background color.
	glColor3f(1, 1, 1);
	glBegin(GL_QUADS);
		glVertex3f(-0.2, -0.25, 0);
		glVertex3f(1.1, -0.25, 0);
		glVertex3f(1.1, 1.2, 0);
		glVertex3f(-0.2, 1.2, 0);
	glEnd();

	//Scale the graph.
	float scale = 1 / (float)(graphData.size() - 1);

	//Draw axis number positions.
	float length = 0.02;
	glColor3f(0, 0, 0);
	glBegin(GL_LINES);
	for (float i = 0; i <= 1.1; i += 0.1) {
		glVertex3f(0, i, 0);
		glVertex3f(-length, i, 0);
		glVertex3f(i, 0, 0);
		glVertex3f(i, -length, 0);
	}
	glEnd();

	//Draw axis position overlay.
	glColor3f(0.75, 0.75, 0.75);
	glBegin(GL_LINES);
	for (float i = 0.1; i <= 1.1; i += 0.1) {
		glVertex3f(0, i, 0);
		glVertex3f(1, i, 0);
		glVertex3f(i, 0, 0);
		glVertex3f(i, 1, 0);
	}
	glEnd();

	//Set line width for data.
	glLineWidth(2);

	//Plot time (x) vs. accuracy (y)
	glColor3f(0, 0, 1);
	glBegin(GL_LINE_STRIP);
	int time = 0;
	for (list<pair<float, float>>::iterator i = graphData.begin(); i != graphData.end(); i++, time++)
		glVertex3f((float)time * scale, i->second, 0);
	glEnd();

	//Plot time (x) vs. completeness (y)
	glColor3f(1, 0, 0);
	glBegin(GL_LINE_STRIP);
	time = 0;
	for (list<pair<float, float>>::iterator i = graphData.begin(); i != graphData.end(); i++, time++)
		glVertex3f((float)time * scale, i->first, 0);
	glEnd();

	//Reset line width.
	glLineWidth(1);

	//Draw axis
	glColor3f(0, 0, 0);
	glBegin(GL_LINE_STRIP);
		glVertex3f(1, 0, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 1, 0);
	glEnd();

	//Draw the y-axis text.
	for (float i = 0; i <= 1.1; i += 0.1) {
		stringstream ss;
		ss << int(i * 100) << "%";
		string str = ss.str();

		glRasterPos2f(-0.15, i - 0.014);                             //text height
		for (int j = 0; j < (int)str.length(); j++)
			glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[j]);
	}

	//Draw the x-axis text.
	for (float i = 0; i <= 1.1; i += 0.2) {
		stringstream ss;
		ss << (graphData.size() - 1) * i;
		string str = ss.str();

		glRasterPos2f(i - (int)str.length() * 0.013, -0.09);
		for (int j = 0; j < (int)str.length(); j++)
			glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[j]);
	}

	//Draw x-axis label.
	char *str = "Elapsed Time (s)";
	glRasterPos2f(0.5 - (int)strlen(str) * 0.013, -0.18);
	for (int j = 0; j < (int)strlen(str); j++)
		glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[j]);

	//Draw graph title.
	glColor3f(0, 0, 0);
	str = "Accuracy vs. Completeness over time";
	glRasterPos2f(0.5 - (int)strlen(str) * 0.013, 1.08);
	for (int j = 0; j < (int)strlen(str); j++)
		glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[j]);

	//Overwrite Accuracy in blue.
	glColor3f(0, 0, 1);
	str = "Accuracy";
	glRasterPos2f(0.045, 1.08);
	for (int j = 0; j < (int)strlen(str); j++)
		glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[j]);

	//Overwrite Completeness in red.
	glColor3f(1, 0, 0);
	str = "Completeness";
	glRasterPos2f(0.382, 1.08);
	for (int j = 0; j < (int)strlen(str); j++)
		glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[j]);
		

	glutSwapBuffers();
}

//Draws a quadrant for strategy 3 display item.
void drawQuadrant(int x, int y) {
	double incrX = (DisplayB->grid.xTo - DisplayB->grid.xFrom) / DisplayB->quadW;
	double incrY = (DisplayB->grid.yTo - DisplayB->grid.yFrom) / DisplayB->quadH;
	glVertex3f(DisplayB->grid.xFrom + x * incrX + DisplayR->startLocation.x, DisplayB->grid.yFrom + y * incrY + DisplayR->startLocation.y, 0);
	glVertex3f(DisplayB->grid.xFrom + (x + 1) * incrX + DisplayR->startLocation.x, DisplayB->grid.yFrom + y * incrY + DisplayR->startLocation.y, 0);
	glVertex3f(DisplayB->grid.xFrom + (x + 1) * incrX + DisplayR->startLocation.x, DisplayB->grid.yFrom + (y + 1) * incrY + DisplayR->startLocation.y, 0);
	glVertex3f(DisplayB->grid.xFrom + x * incrX + DisplayR->startLocation.x, DisplayB->grid.yFrom + (y + 1) * incrY + DisplayR->startLocation.y, 0);
}

void displayCallback() {
	glClear(GL_COLOR_BUFFER_BIT);

	//Draw the environment.
	//Background.
	glColor3f(1, 1, 1);
	glBegin(GL_POLYGON);
		glVertex3f(-0.01f * DisplayE->width, -0.01f * DisplayE->height, 0);
		glVertex3f(-0.01f * DisplayE->width, 1.01f * DisplayE->height, 0);
		glVertex3f(1.01f * DisplayE->width, 1.01f * DisplayE->height, 0);
		glVertex3f(1.01f * DisplayE->width, -0.01f * DisplayE->height, 0);
	glEnd();

	//Draw the grid model.
	if (gridE) {
		for (int y = 0; y < DisplayB->grid.height; y++)
			for (int x = 0; x < DisplayB->grid.width; x++) {
				//Calculate cell corner locations.
				double blX = DisplayB->grid.xFrom + (x - 1) * DisplayB->grid.curGran;
				double blY = DisplayB->grid.yFrom + (y - 1) * DisplayB->grid.curGran;
				double brX = DisplayB->grid.xFrom + x * DisplayB->grid.curGran;
				double brY = DisplayB->grid.yFrom + (y - 1) * DisplayB->grid.curGran;
				double trX = DisplayB->grid.xFrom + x * DisplayB->grid.curGran;
				double trY = DisplayB->grid.yFrom + y * DisplayB->grid.curGran;
				double tlX = DisplayB->grid.xFrom + (x - 1) * DisplayB->grid.curGran;
				double tlY = DisplayB->grid.yFrom + y * DisplayB->grid.curGran;

				//Base the cell colour on point probability.
				double color = DisplayB->grid.cells[x][y];
				glColor3f(1, 1 - color, 1 - color);

				glBegin(GL_QUADS);
				glVertex3f(blX + DisplayR->startLocation.x, blY + DisplayR->startLocation.y, 0);
				glVertex3f(brX + DisplayR->startLocation.x, brY + DisplayR->startLocation.y, 0);
				glVertex3f(trX + DisplayR->startLocation.x, trY + DisplayR->startLocation.y, 0);
				glVertex3f(tlX + DisplayR->startLocation.x, tlY + DisplayR->startLocation.y, 0);
				glEnd();
			}
	}

	//Draw strategy 3's quadrants.
	if (quadE && (DisplayB->strategy == 3 || DisplayB->strategy == 4)) {
		//Neighbour quadrants.
		int min = 999999, max = -1;
		for (int y = DisplayB->curY - 1; y <= DisplayB->curY + 1; y++)
			for (int x = DisplayB->curX - 1; x <= DisplayB->curX + 1; x++) {
				//Check bounds.
				if (x < 0 || y < 0) continue;
				if (x >= DisplayB->quadW || y >= DisplayB->quadH) continue;
				//Skip current and diagonal.
				if (x == DisplayB->curX && y == DisplayB->curY) continue;
				if (x != DisplayB->curX && y != DisplayB->curY) continue;
				//Find the min and max.
				if (x >= DisplayB->quadW || y >= DisplayB->quadH) continue;
				if (DisplayB->quadrant[x][y] < min) min = DisplayB->quadrant[x][y];
				if (DisplayB->quadrant[x][y] > max) max = DisplayB->quadrant[x][y];
			}
		glBegin(GL_QUADS);
		for (int y = DisplayB->curY - 1; y <= DisplayB->curY + 1; y++)
			for (int x = DisplayB->curX - 1; x <= DisplayB->curX + 1; x++) {
				if (x < 0 || y < 0) continue;
				if (x >= DisplayB->quadW || y >= DisplayB->quadH) continue;
				//[//] Don't skip current cell - shows comparison to neighbours.
				//if (x == DisplayB->curX && y == DisplayB->curY) continue;
				if (x != DisplayB->curX && y != DisplayB->curY) continue;
				//Colour neighbours according to min and max.
				float grad = float(DisplayB->quadrant[x][y] - min) / (max - min);
				//Clamp colour incase current cell exceeds expected maximum.
				if (grad < 0) grad = 0; if (grad > 1) grad = 1;
				glColor4f(1, 1, 0, (1 - grad) * 0.25);
				drawQuadrant(x, y);
			}
		glEnd();

		//Borders.
		for (int y = 0; y < DisplayB->quadH; y++)
			for (int x = 0; x < DisplayB->quadW; x++) {
				glBegin(GL_LINE_LOOP);
				glColor3f(0.8, 0.8, 0.8);
				drawQuadrant(x, y);
				glEnd();
			}
	}

	//Draw the grid-robot overlay.
	if (overlayE) {
		glColor3f(0.5, 0.5, 1);
		glBegin(GL_QUADS);
		for (set<Vertex>::iterator i = DisplayB->gridOverlay.begin(); i != DisplayB->gridOverlay.end(); i++) {
			glVertex3f(DisplayB->grid.worldX(i->x) + DisplayR->startLocation.x, DisplayB->grid.worldY(i->y) + DisplayR->startLocation.y, 0);
			glVertex3f(DisplayB->grid.worldX(i->x + 1) + DisplayR->startLocation.x, DisplayB->grid.worldY(i->y) + DisplayR->startLocation.y, 0);
			glVertex3f(DisplayB->grid.worldX(i->x + 1) + DisplayR->startLocation.x, DisplayB->grid.worldY(i->y + 1) + DisplayR->startLocation.y, 0);
			glVertex3f(DisplayB->grid.worldX(i->x) + DisplayR->startLocation.x, DisplayB->grid.worldY(i->y + 1) + DisplayR->startLocation.y, 0);
		}
		glEnd();
	}

	//Draw the cells undergoing a collision check.
	if (collisionE) {
		glColor3f(1, 0.5, 1);
		glBegin(GL_QUADS);
		for (set<Vertex>::iterator i = DisplayB->difference.begin(); i != DisplayB->difference.end(); i++) {
			glVertex3f(DisplayB->grid.worldX(i->x) + DisplayR->startLocation.x, DisplayB->grid.worldY(i->y) + DisplayR->startLocation.y, 0);
			glVertex3f(DisplayB->grid.worldX(i->x + 1) + DisplayR->startLocation.x, DisplayB->grid.worldY(i->y) + DisplayR->startLocation.y, 0);
			glVertex3f(DisplayB->grid.worldX(i->x + 1) + DisplayR->startLocation.x, DisplayB->grid.worldY(i->y + 1) + DisplayR->startLocation.y, 0);
			glVertex3f(DisplayB->grid.worldX(i->x) + DisplayR->startLocation.x, DisplayB->grid.worldY(i->y + 1) + DisplayR->startLocation.y, 0);
		}
		glEnd();
	}

	//Draw the wall tracker.
	if (trackerE && DisplayB->strategy == 1) {
		glColor3f(1, 1, 0);
		glBegin(GL_QUADS);
		for (set<Vertex>::iterator i = DisplayB->intersectCells.begin(); i != DisplayB->intersectCells.end(); i++) {
			glVertex3f(DisplayB->grid.worldX(i->x - 1) + DisplayR->startLocation.x, DisplayB->grid.worldY(i->y - 1) + DisplayR->startLocation.y, 0);
			glVertex3f(DisplayB->grid.worldX(i->x) + DisplayR->startLocation.x, DisplayB->grid.worldY(i->y - 1) + DisplayR->startLocation.y, 0);
			glVertex3f(DisplayB->grid.worldX(i->x) + DisplayR->startLocation.x, DisplayB->grid.worldY(i->y) + DisplayR->startLocation.y, 0);
			glVertex3f(DisplayB->grid.worldX(i->x - 1) + DisplayR->startLocation.x, DisplayB->grid.worldY(i->y) + DisplayR->startLocation.y, 0);
		}
		glEnd();
	}

	//Draw obstacles.
	if (obstaclesE) {
		glColor3f(0, 0, 0);
		for (std::list<Polygon>::iterator i = DisplayE->obstacles.begin(); i != DisplayE->obstacles.end(); i++) {
			glBegin(GL_POLYGON);
			for (std::list<Vertex>::iterator j = i->vertices.begin(); j != i->vertices.end(); j++)
				glVertex3f(j->x, j->y, 0);
			glEnd();
		}
	}

	//Draw beacons.
	if (beaconsE) {
		glColor3f(0, 0, 0.5);
		for (std::list<Polygon>::iterator i = DisplayE->beacons.begin(); i!= DisplayE->beacons.end(); i++) {
			glBegin(GL_POLYGON);
			for (std::list<Vertex>::iterator j = i->vertices.begin(); j != i->vertices.end(); j++)
				glVertex3f(j->x, j->y, 0);
			glEnd();
		}
	}

	//Draw data vertices.
	if (verticesE) {
		glBegin(GL_POINTS);
		glColor3f(1, 0, 0);
		for (unsigned int i = 0; i < DisplayB->data.size(); i++) {
			//Only draw the points that have tolerable variance.
			if (DisplayB->toleranceFilter(DisplayB->data[i])) {
				Vertex v = DisplayB->getVertex(DisplayB->data[i].x, DisplayB->data[i].y, DisplayB->data[i].l, DisplayB->data[i].d);
				glVertex3f(v.x + DisplayR->startLocation.x, v.y + DisplayR->startLocation.y, 0);
			}
		}
		glEnd();
	}

	//Draw the behaviour path.
	if (pathE) {
		glColor3f(0, 1, 0);
		glBegin(GL_LINE_STRIP);
		for (unsigned int i = 0; i < DisplayB->data.size(); i++)
			glVertex3f(DisplayB->data[i].x + DisplayR->startLocation.x, DisplayB->data[i].y + DisplayR->startLocation.y, 0);
		glEnd();
	}

	//Draw strategy 3's, seek line.
	if (seekE && (DisplayB->strategy == 3 || DisplayB->strategy == 4)) {
		glLineWidth(2);
		glColor3f(0.5, 0.5, 0.5);
		glBegin(GL_LINES);
		glVertex3f(DisplayB->location.x + DisplayR->startLocation.x, DisplayB->location.y + DisplayR->startLocation.y, 0);
		glVertex3f(DisplayB->quadX + DisplayR->startLocation.x, DisplayB->quadY + DisplayR->startLocation.y, 0);
		glEnd();
		glLineWidth(1);
	}

	//Draw the robot.
	if (robotE) {
		glColor3f(0, 0, 1);
		glBegin(GL_POLYGON);
		glVertex3f(DisplayR->frontLeft.x, DisplayR->frontLeft.y, 0);
		glVertex3f(DisplayR->frontRight.x, DisplayR->frontRight.y, 0);
		glVertex3f(DisplayR->backRight.x, DisplayR->backRight.y, 0);
		glVertex3f(DisplayR->backLeft.x, DisplayR->backLeft.y, 0);
		glEnd();

		//Draw robot direction line.
		glColor3f(0.5, 0.5, 1);
		glBegin(GL_LINES);
		//Centre line
		glVertex3f((DisplayR->frontLeft.x + DisplayR->frontRight.x) / 2, (DisplayR->frontLeft.y + DisplayR->frontRight.y) / 2, 0);
		glVertex3f((DisplayR->backLeft.x + DisplayR->backRight.x) / 2, (DisplayR->backLeft.y + DisplayR->backRight.y) / 2, 0);
		//Left tip
		glVertex3f((DisplayR->frontLeft.x + DisplayR->frontRight.x) / 2, (DisplayR->frontLeft.y + DisplayR->frontRight.y) / 2, 0);
		glVertex3f((3 * DisplayR->frontLeft.x + DisplayR->backLeft.x) / 4, (3 * DisplayR->frontLeft.y + DisplayR->backLeft.y) / 4, 0);
		//Right tip
		glVertex3f((DisplayR->frontLeft.x + DisplayR->frontRight.x) / 2, (DisplayR->frontLeft.y + DisplayR->frontRight.y) / 2, 0);
		glVertex3f((3 * DisplayR->frontRight.x + DisplayR->backRight.x) / 4, (3 * DisplayR->frontRight.y + DisplayR->backRight.y) / 4, 0);
		glEnd();
	}

	//Draw the variance ellipses.
	if (ellipsesE) {
		//1 Standard deviation.
		glColor3f(0, 1, 0);
		glBegin(GL_LINE_LOOP);
		for (int i = 0; i < 360; i++) {
			float xSD = cos(i * (float)PI / 180) * sqrt(DisplayB->xV);
			float ySD = sin(i * (float)PI / 180) * sqrt(DisplayB->yV);
			glVertex3f(xSD + DisplayB->location.x + DisplayR->startLocation.x, ySD + DisplayB->location.y + DisplayR->startLocation.y, 0);
		}
		glEnd();
		//2 Standard deviations.
		glColor3f(0, 0.75, 0);
		glBegin(GL_LINE_LOOP);
		for (int i = 0; i < 360; i++) {
			float xSD = 2 * cos(i * (float)PI / 180) * sqrt(DisplayB->xV);
			float ySD = 2 * sin(i * (float)PI / 180) * sqrt(DisplayB->yV);
			glVertex3f(xSD + DisplayB->location.x + DisplayR->startLocation.x, ySD + DisplayB->location.y + DisplayR->startLocation.y, 0);
		}
		glEnd();
		//3 Standard deviations.
		glColor3f(0, 0.5, 0);
		glBegin(GL_LINE_LOOP);
		for (int i = 0; i < 360; i++) {
			float xSD = 3 * cos(i * (float)PI / 180) * sqrt(DisplayB->xV);
			float ySD = 3 * sin(i * (float)PI / 180) * sqrt(DisplayB->yV);
			glVertex3f(xSD + DisplayB->location.x + DisplayR->startLocation.x, ySD + DisplayB->location.y + DisplayR->startLocation.y, 0);
		}
		glEnd();
	}

	//Draw the lidar.
	if (lidarE) {
		double rad = DisplayR->lidarAngle * PI / 180;
		double c = DisplayR->lidarDistance;
		glColor3f(1, 0, 0);
		glBegin(GL_LINES);
		glVertex3f(DisplayR->location.x, DisplayR->location.y, 0);
		glVertex3f(DisplayR->location.x + c * cos(rad), DisplayR->location.y + c * sin(rad), 0);
		glEnd();
	}

	//Draw the overlay-lidar.
	if (detectorE) {
		double rad = DisplayR->lidarAngle * PI / 180;
		double c = DisplayR->lidarDistance;
		glColor3f(0, 0, 1);
		glBegin(GL_LINES);
		glVertex3f(DisplayB->location.x + DisplayR->startLocation.x, DisplayB->location.y + DisplayR->startLocation.y, 0);
		glVertex3f(DisplayB->location.x + c * cos(rad) + DisplayR->startLocation.x, DisplayB->location.y + c * sin(rad) + DisplayR->startLocation.y, 0);
		glEnd();
	}

	//Draw data lines.
	if (mappingsE) {
		glBegin(GL_LINES);
		glColor3f(0.9, 0.9, 0.9);
		for (unsigned int i = 0; i < DisplayB->data.size(); i++) {
			//Only draw the points that have tolerable variance.
			if (DisplayB->toleranceFilter(DisplayB->data[i])) {
				Vertex v = DisplayB->getVertex(DisplayB->data[i].x, DisplayB->data[i].y, DisplayB->data[i].l, DisplayB->data[i].d);
				glVertex3f(DisplayB->data[i].x + DisplayR->startLocation.x, DisplayB->data[i].y + DisplayR->startLocation.y, 0);
				glVertex3f(v.x + DisplayR->startLocation.x, v.y + DisplayR->startLocation.y, 0);
			}
		}
		glEnd();
	}
	
	//Code block reserved for debugging.
	if (debugE) {
		glColor3f(0, 0, 1);
		glBegin(GL_LINE_STRIP);
		for (unsigned int i = 0; i < DisplayB->data.size(); i++) {
			Vertex v = DisplayB->getVertex(DisplayB->data[i].x, DisplayB->data[i].y, DisplayB->data[i].l, DisplayB->data[i].d);
			glVertex3f(v.x + DisplayR->startLocation.x, v.y + DisplayR->startLocation.y, 0);
		}
		glEnd();
	}

	glutSwapBuffers();
}

void graphTimer(int n) {
	glutPostWindowRedisplay(graphWindow);
	glutTimerFunc(1000 / n, graphTimer, n);
}

/*
Allows the behaviour class to take a turn, then repaints the display.
The wait time between frames is fixed by checking how long it takes to render each frame.
*/
void timerCallback(int n) {
	if (!MENU_OPEN && !PAUSED) {
		double elapsed = (double)(glutGet(GLUT_ELAPSED_TIME) - lastTime) / 1000;

		//If the elapsed time is greater than the pauseTimeout then skip this frame.
		//We assume that the window was dragged and therefore do not with to procees the animation for those frames.
		if (elapsed <= (double)(1000 / n + pauseTimeout) / 1000) {
			DisplayB->nextMove(elapsed);
			DisplayB->nextLidar(elapsed);

			//Set window title with run time.
			runtime += elapsed;
			ostringstream ss;
			ss << "Simulated Autonomous Exploration | Christopher Patuzzo | Elapsed: " << runtime << "s";
			glutSetWindow(mainWindow);
			glutSetWindowTitle(ss.str().c_str());

			//Check if end of animation reached.
			if (timeLimit) {
				if (runtime >= maxTime || DisplayB->stuck) {
					//Write grid values to file.
					fstream out("out.txt", fstream::in | fstream::out | fstream::app);
					static bool meta = false;
					if (!meta) {
						out << filename << endl;
						meta = true;
					}

					//Print exit state.
					if (runtime >= maxTime)
						out << "0";
					else out << runtime;

					//Print completeness and accuracy.
					out << "\t" << DisplayB->grid.completeness();
					out << "\t" << DisplayB->grid.accuracy() << endl;

					//Close file and next test.
					out.close();
					nextTest();
				}
			}
		}

		lastTime = glutGet(GLUT_ELAPSED_TIME);

		//Update the graph data per second.
		if (runtime >= graphIndex) {
			pair<float, float> datum;
			datum.first = DisplayB->grid.completeness();
			datum.second = DisplayB->grid.accuracy();

			if (datum.first > 0.000001);
			else datum.first = 0;
			if (datum.second > 0.000001);
			else datum.second = 0;

			graphData.push_back(datum);
			graphIndex++;
		}
	}
	//Redisplay outside of menu and pause check so that views may still be toggled.
	glutPostWindowRedisplay(mainWindow);

	glutTimerFunc(1000 / n, timerCallback, n);
}

void keyboardCallback(unsigned char key, int mousex, int mousey) {
	//Real
	if (key == 'r' || key == 'R')
		robotE = !robotE;
	else if (key == 'l' || key == 'L')
		lidarE = !lidarE;
	else if (key == 'o' || key == 'O')
		obstaclesE = !obstaclesE;
	else if (key == 'b' || key == 'B')
		beaconsE = !beaconsE;
	//Internalised
	else if (key == 'v' || key == 'V')
		overlayE = !overlayE;
	else if (key == 'c' || key == 'C')
		collisionE = !collisionE;
	else if (key == 'd' || key == 'D')
		detectorE = !detectorE;
	else if (key == 'e' || key == 'E')
		ellipsesE = !ellipsesE;
	else if (key == 'p' || key == 'P')
		pathE = !pathE;
	//Data
	else if (key == 'g' || key == 'G')
		gridE = !gridE;
	else if (key == 'm' || key == 'M')
		mappingsE = !mappingsE;
	else if (key == 't' || key == 'T')
		verticesE = !verticesE;
	//Strategies
	else if (key == 'a' || key == 'A')
		trackerE = !trackerE;
	else if (key == 'u' || key == 'U')
		quadE = !quadE;
	else if (key == 's' || key == 'S')
		seekE = !seekE;
	//Misc
	else if (key == 'x' || key == 'X')
		debugE = !debugE;
	else if (key == ' ')
		PAUSED = !PAUSED;
	else if (key == 'q' || key == 'Q')
		exit(0);

	setMenuStrings();
}

//Disable reshape and maximise for graph window.
//This is necessary due to fixed size bitmap fonts.
void graphReshape(int w , int h) {
	glutReshapeWindow(450, 450);
	glutPositionWindow(windowWidth + 40, (windowHeight - 450) / 2);
}

//Called when the window is reshaped. Maintains aspect ratio.
void reshapeCallback(int width, int height) {
	double windowAspect = (double)width / height;
	double modelAspect = (double)DisplayE->width / DisplayE->height;
	double offset;

	if (modelAspect > windowAspect) {
		offset = (1 - windowAspect / modelAspect) * height;
		xFrom = 0; xTo = width;
		yFrom = offset / 2; yTo = height - yFrom;
	}
	else {
		offset = (1 - modelAspect / windowAspect) * width;
		xFrom = offset / 2; xTo = width - xFrom;
		yFrom = 0; yTo = height;
	}
	
	glViewport(xFrom, yFrom, xTo - xFrom, yTo - yFrom);
}

void menuStatusCallback(int status) {
	if (status == GLUT_MENU_IN_USE)
		MENU_OPEN = true;
	else
		MENU_OPEN = false;
}

void menuCallback(int item) {
	switch (item) {
		//Real
		case VIEW_ROBOT:
			robotE = !robotE;
			break;
		case VIEW_LIDAR:
			lidarE = !lidarE;
			break;
		case VIEW_OBSTACLES:
			obstaclesE = !obstaclesE;
			break;
		case VIEW_BEACONS:
			beaconsE = !beaconsE;
			break;
		//Internalised
		case VIEW_OVERLAY:
			overlayE = !overlayE;
			break;
		case VIEW_COLLISION:
			collisionE = !collisionE;
			break;
		case VIEW_DETECTOR:
			detectorE = !detectorE;
			break;
		case VIEW_ELLIPSES:
			ellipsesE = !ellipsesE;
			break;
		case VIEW_PATH:
			pathE = !pathE;
			break;
		//Data
		case VIEW_GRID:
			gridE = !gridE;
			break;
		case VIEW_MAPPINGS:
			mappingsE = !mappingsE;
			break;
		case VIEW_VERTICES:
			verticesE = !verticesE;
			break;
		//Strategies
		case VIEW_TRACKER:
			trackerE = !trackerE;
			break;
		case VIEW_QUADRANTS:
			quadE = !quadE;
			break;
		case VIEW_SEEK:
			seekE = !seekE;
			break;
		//Misc
		case VIEW_DEBUG:
			debugE = !debugE;
			break;
		case MENU_PAUSE:
			PAUSED = !PAUSED;
			break;
		case MENU_EXIT:
			exit(0);
			break;
	}

	setMenuStrings();
}

void setMenuStrings() {
	glutChangeToMenuEntry(VIEW_ROBOT, (robotE ? "[ON] Robot [R]" : "[OFF] Robot [R]"), VIEW_ROBOT);
	glutChangeToMenuEntry(VIEW_LIDAR, (lidarE ? "[ON] LIDAR [L]" : "[OFF] LIDAR [L]"), VIEW_LIDAR);
	glutChangeToMenuEntry(VIEW_OBSTACLES, (obstaclesE ? "[ON] Obstacles [O]" : "[OFF] Obstacles [O]"), VIEW_OBSTACLES);
	glutChangeToMenuEntry(VIEW_BEACONS, (beaconsE ? "[ON] Beacons [B]" : "[OFF] Beacons [B]"), VIEW_BEACONS);
	
	glutChangeToMenuEntry(VIEW_OVERLAY, (overlayE ? "[ON] Overlay [V]" : "[OFF] Overlay [V]"), VIEW_OVERLAY);
	glutChangeToMenuEntry(VIEW_COLLISION, (collisionE ? "[ON] Collision [C]" : "[OFF] Collision [C]"), VIEW_COLLISION);
	glutChangeToMenuEntry(VIEW_DETECTOR, (detectorE ? "[ON] Detector [D]" : "[OFF] Detector [D]"), VIEW_DETECTOR);
	glutChangeToMenuEntry(VIEW_ELLIPSES, (ellipsesE ? "[ON] Ellipses [E]" : "[OFF] Ellipses [E]"), VIEW_ELLIPSES);
	glutChangeToMenuEntry(VIEW_PATH, (pathE ? "[ON] Path [P]" : "[OFF] Path [P]"), VIEW_PATH);

	glutChangeToMenuEntry(VIEW_GRID, (gridE ? "[ON] Grid [G]" : "[OFF] Grid [G]"), VIEW_GRID);
	glutChangeToMenuEntry(VIEW_MAPPINGS, (mappingsE ? "[ON] Mappings [M]" : "[OFF] Mappings [M]"), VIEW_MAPPINGS);
	glutChangeToMenuEntry(VIEW_VERTICES, (verticesE ? "[ON] Vertices [T]" : "[OFF] Vertices [T]"), VIEW_VERTICES);

	glutChangeToMenuEntry(VIEW_TRACKER, (trackerE ? "[ON] 1: Tracker [A]" : "[OFF] 1: Tracker [A]"), VIEW_TRACKER);
	glutChangeToMenuEntry(VIEW_QUADRANTS, (quadE ? "[ON] 3, 4: Quadrants [U]" : "[OFF] 3, 4: Quadrants [U]"), VIEW_QUADRANTS);
	glutChangeToMenuEntry(VIEW_SEEK, (seekE ? "[ON] 3, 4: Seeking [S]" : "[OFF] 3, 4: Seeking [S]"), VIEW_SEEK);

	glutChangeToMenuEntry(VIEW_DEBUG, (debugE ? "[ON] Debug [X]" : "[OFF] Debug [X]"), VIEW_DEBUG);
	glutChangeToMenuEntry(MENU_PAUSE, (PAUSED ? "Unpause [SPACE]" : "Pause [SPACE]"), MENU_PAUSE);
	glutChangeToMenuEntry(MENU_EXIT, "Exit [Q]", MENU_EXIT);
}

#endif