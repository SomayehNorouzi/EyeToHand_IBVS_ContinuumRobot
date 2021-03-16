#ifndef CRSimulation_h
#define CRSimulation_h

/*!

\file vpViper.h

Modelisation of the ADEPT Viper 650 or 850 robot.

*/

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/robot/vpRobotException.h>

class CRSimulation
{
public:
	CRSimulation();
	virtual ~CRSimulation() {};

	
	vpHomogeneousMatrix get_fMe(const vpColVector &q);
	
	void get_fMw(const vpColVector &q, vpHomogeneousMatrix &fMw);
	void get_wMe(vpHomogeneousMatrix &wMe);
	void get_fJw(const vpColVector &q, vpMatrix &fJw);
	void get_fJe(const vpColVector &q, vpMatrix &fJe);
	void get_eJe(const vpColVector &q, vpMatrix &eJe);
	void get_q(const vpColVector &qdot, vpColVector &q);
	

	void setSamplingTime(double SamplingTime);
	double getSamplingTime();

	
	friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const NewVpViper &viper);
public:
	static const unsigned int njoint; ///< Number of joint.

protected:

// Denavit-Hartenberg parameters
	double L; 
	double d7; 
	double DK;
	double RC;
	double K;
	double _SamplingTime;
	
};

#endif#pragma once
