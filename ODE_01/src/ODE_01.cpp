//============================================================================
// Name        : ODE_01.cpp
// Author      : David Eichmann
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <ode/ode.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>

#include "Constants.h"


void stepWorld() {

	//	Loop:
	//		Apply forces to the bodies as necessary.
	//		Adjust the joint parameters as necessary.
	//		Call collision detection.
	//		Create a contact joint for every collision point, and put it in the contact joint group.
	//		Take a simulation step.
	//		Remove all joints in the contact joint group.

}

void renderScene() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glutSolidSphere(0.7,20,20);

	glutSwapBuffers();
}

void keyHandler(unsigned char c, int a, int b) {
	switch(c) {
	case 27:
		glutExit();
		break;
	}
}

void destroyWorld(dWorldID wid) {

	//	Destroy the dynamics and collision worlds.
	dWorldDestroy (wid);

}

dWorldID initWorld() {

	//	Create a dynamics world.
	dWorldID id = dWorldCreate();
	/// gravity
	dWorldSetGravity(id, 0, GRAVITY_ACC, 0);
	/// space
	const dSpaceID space = dHashSpaceCreate(0);
	/// floor
	dCreatePlane(space,0,1,0,0);

	//	Create bodies in the dynamics world.

	//	Set the state (position etc) of all bodies.

	//	Create joints in the dynamics world.

	//	Attach the joints to the bodies.

	//	Set the parameters of all joints.

	//	Create a collision world and collision geometry objects, as necessary.

	//	Create a joint group to hold the contact joints.

	return id;
}

int main(int pargc, char** argv) {

	// initialize glut
	glutInit(&pargc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(800, 800);
	glutCreateWindow("My awesome 3D program");
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);

	// register callbacks
	glutDisplayFunc(renderScene);
	glutKeyboardFunc(keyHandler);

	// setup world

	// lighting
	glClearColor(1,1,1,1);
	glEnable(GL_LIGHT0);
	GLfloat light_point[] = { 1,1,-1, 0 };
	glLightfv(GL_LIGHT0, GL_POSITION, light_point);

	glEnable(GL_LIGHT1);
	GLfloat light_ambient[] = { 0.3, 0.3, 0.3, 1.0 };
	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);

	// world contents
	dInitODE();
	dWorldID wid = initWorld();

	// enter GLUT event processing cycle
	glutMainLoop();

	// destroy world
	destroyWorld(wid);

	return 1;

}
