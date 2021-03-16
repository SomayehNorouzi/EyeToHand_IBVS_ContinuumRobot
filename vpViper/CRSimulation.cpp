
/****************************************************************************
*
* Description:
* Interface for a  generic CR robot.
*
* Authors:Somayhe Norouzi 
*
*****************************************************************************/

#include "stdafx.h"
#include <visp/vpDebug.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpRobotException.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRotationMatrix.h>
#include "CRSimulation.h"
#include <cmath>    
#include <limits>   


const unsigned int CRSimulation::njoint = 3; 

CRSimulation::CRSimulation()
{
	// Denavit Hartenberg parameters
	d7 = 0; // if getting a value more than 0, must be updated in get_wMe
	L = 0.07;
	DK = 0.011; //Knob diameter
	RC = 0.0145;//radius of fixed curvature
	K = 1;

 }


/*!
Compute the forward kinematics (direct geometric model) as an
homogeneous matrix
*/
vpHomogeneousMatrix
CRSimulation::get_fMe(const vpColVector& q)
{
	double q1 = q[0];//t2
	double q2 = q[1];//t3
	double q3 = q[2];//d1


	double c1 = cos(q1);
	double s1 = sin(q1);
	double c2 = cos(q2);
	double s2 = sin(q2);

	double cd1 = cos(2 * q1);
	double sd1 = sin(2 * q1);
	double cd2 = cos(2 * q2);
	double sd2 = sin(2 * q2);

	vpHomogeneousMatrix fMe; //(07/17/2020)

	fMe[0][0] = cd2 * c1;
	fMe[1][0] = cd2 * s1;
	fMe[2][0] = -sd2;
	fMe[3][0] = 0;

	fMe[0][1] = s1;
	fMe[1][1] = -c1;
	fMe[2][1] = 0;
	fMe[3][1] = 0;

	fMe[0][2] = -sd2 * c1;
	fMe[1][2] = -sd2 * s1;
	fMe[2][2] = -cd2;
	fMe[3][2] = 0;

	fMe[0][3] = (2 * L * c1 * c2 * c2) / (2 * q2 - M_PI);
	fMe[1][3] = (2 * L * c2 * c2 * s1) / (2 * q2 - M_PI);
	fMe[2][3] = q3 - (L * sd2 / (2 * q2 - M_PI));
	fMe[3][3] = 1;

	return fMe;
}
/*!
Compute the transformation between the fix frame and the wrist frame.
*/
void
CRSimulation::get_fMw(const vpColVector& q, vpHomogeneousMatrix& fMw)
{
	double q1 = q[0];//t2
	double q2 = q[1];//t3
	double q3 = q[2];//d1


	double c1 = cos(q1);
	double s1 = sin(q1);
	double c2 = cos(q2);
	double s2 = sin(q2);

	double cd1 = cos(2 * q1);
	double sd1 = sin(2 * q1);
	double cd2 = cos(2 * q2);
	double sd2 = sin(2 * q2);



	fMw[0][0] = cd2 * c1;
	fMw[1][0] = cd2 * s1;
	fMw[2][0] = -sd2;
	fMw[3][0] = 0;

	fMw[0][1] = s1;
	fMw[1][1] = -c1;
	fMw[2][1] = 0;
	fMw[3][1] = 0;

	fMw[0][2] = -sd2 * c1;
	fMw[1][2] = -sd2 * s1;
	fMw[2][2] = -cd2;
	fMw[3][2] = 0;

	fMw[0][3] = (2 * L * c1 * c2 * c2) / (2 * q2 - M_PI);
	fMw[1][3] = (2 * L * c2 * c2 * s1) / (2 * q2 - M_PI);
	fMw[2][3] = q3 - (L * sd2 / (2 * q2 - M_PI));
	fMw[3][3] = 1;

	// std::cout << "Effector position fMe: " << std::endl << fMe;

	return;
}

/*!
Return the transformation between the wrist frame and the end-effector. The
wrist frame is located on the intersection of the 3 last rotations.
\param wMe The homogeneous matrix corresponding to the transformation between
the wrist frame and the end-effector frame (wMe).
*/
void
CRSimulation::get_wMe(vpHomogeneousMatrix& wMe)
{
	// Set the rotation as identity
	wMe.eye();

	// Set the translation
	wMe[0][3] = 0;
	wMe[1][3] = 0;
	wMe[2][3] = 0;


}

/*!
Get the robot jacobian which gives the velocity
of the origin of the end-effector frame expressed in end-effector frame.
param q : A six-dimension vector that contains the joint positions
of the robot expressed in radians.

param eJe : Robot jacobian that express the
velocity of the end-effector in the robot end-effector frame.
eVe= eJe * qdot {in reference frame}

*/
void
CRSimulation::get_eJe(const vpColVector& q, vpMatrix& eJe)
{
	vpMatrix V(6, 6);
	V = 0;
	// Compute the first and last block of V
	vpHomogeneousMatrix fMw;
	get_fMw(q, fMw);
	vpRotationMatrix fRw;
	fMw.extract(fRw);
	vpRotationMatrix wRf;
	wRf = fRw.inverse();
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++) {
			V[i][j] = V[i + 3][j + 3] = wRf[i][j];
		}
	}
	// Compute the second block of V
	vpHomogeneousMatrix wMe;
	get_wMe(wMe);
	vpHomogeneousMatrix eMw;
	eMw = wMe.inverse();
	vpTranslationVector etw;
	eMw.extract(etw);
	vpMatrix block2 = etw.skew() * wRf;
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++) {
			V[i][j + 3] = block2[i][j];
		}
	}
	// Compute eJe
	vpMatrix fJw;
	get_fJw(q, fJw);
	eJe = V * fJw;

	return;
}


/*!
param q : A six-dimension vector that contains the joint positions
of the robot expressed in radians.

param fJw : Robot jacobian that express the
velocity of the point w (origin of the wrist frame) in the robot
reference frame.
*/

void
CRSimulation::get_fJw(const vpColVector& q, vpMatrix& fJw)
{
	double q1 = q[0];
	double q2 = q[1];
	double q3 = q[2];

	double c1 = cos(q1);
	double s1 = sin(q1);
	double c2 = cos(q2);
	double s2 = sin(q2);

	double cd1 = cos(2 * q1);
	double sd1 = sin(2 * q1);
	double cd2 = cos(2 * q2);
	double sd2 = sin(2 * q2);

	double b = DK / (2 * L * RC);


	// J-Catheter

	fJw.resize(6, 3);

	fJw[0][0] = -2 * L * c2 * c2 * s1 / (2 * q2 - M_PI);
	fJw[1][0] = 2 * L * c1 * c2 * c2 / (2 * q2 - M_PI);
	fJw[2][0] = 0;
	fJw[3][0] = 0;
	fJw[4][0] = 0;
	fJw[5][0] = +1;

	fJw[0][1] = -4 * L * c1 * c2 * (c2 - M_PI * s2 + 2 * q2 * s2) / ((2 * q2 - M_PI) * (2 * q2 - M_PI));
	fJw[1][1] = -4 * L * c2 * s1 * (c2 - M_PI * s2 + 2 * q2 * s2) / ((2 * q2 - M_PI) * (2 * q2 - M_PI));
	fJw[2][1] = 2 * L * (sd2 - 2 * q2 * cd2 + M_PI * cd2) / ((2 * q2 - M_PI) * (2 * q2 - M_PI));
	fJw[3][1] = -2 * s1;
	fJw[4][1] = 2 * c1;
	fJw[5][1] = 0;

	fJw[0][2] = 0;
	fJw[1][2] = 0;
	fJw[2][2] = 1;
	fJw[3][2] = 0;
	fJw[4][2] = 0;
	fJw[5][2] = 0;

	return;
}
/*!
param q : A six-dimension vector that contains the joint positions
of the robot expressed in radians.

param fJe : Robot jacobian that express the
velocity of the end-effector in the robot reference frame.


*/
void
CRSimulation::get_fJe(const vpColVector& q, vpMatrix& fJe)
{
	vpMatrix fJw;
	get_fJw(q, fJw);
	fJe = fJw;

	return;
}


void
CRSimulation::setSamplingTime(double SamplingTime)
{
	_SamplingTime = SamplingTime;
}

double
CRSimulation::getSamplingTime()
{
	return _SamplingTime;
}

void
CRSimulation::get_q(const vpColVector& qdot, vpColVector& q)
{
	q+=_SamplingTime * qdot;
}



