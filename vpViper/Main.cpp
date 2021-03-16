// Eye-To-Hand IBVS

/****************************************************************************
* Somayeh Norouzi:
"This project provides the implementation of Eye-To-Hand IBVS on an ablation
catheter as a continuum robot. 
CRSimulation include the kinematic model of CR.

The program is based on ViSP libraries." 

* ViSP, open source Visual Servoing Platform software.
* Copyright (C) 2005 - 2019 by Inria. All rights reserved.
*
* This software is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* See the file LICENSE.txt at the root directory of this source
* distribution for additional information about the GNU GPL.
*
* For using ViSP with software that can not be combined with the GNU
* GPL, please contact Inria about acquiring a ViSP Professional
* Edition License.
*
* See http://visp.inria.fr for more information.
*
* This software was developed at:
* Inria Rennes - Bretagne Atlantique
* Campus Universitaire de Beaulieu
* 35042 Rennes Cedex
* France
*
* If you have questions regarding the use of this file, please contact
* Inria at visp@inria.fr
*
* This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
* WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*
* Description:
* Interface for a  generic ADEPT Viper (either 650 or 850) robot.
*
* Authors:
* Fabien Spindler
*
*****************************************************************************/

#include "stdafx.h"
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpProjectionDisplay.h>			// allows to handle the external view from a given camera position
#include <visp3/visual_features/vpFeatureBuilder.h> // will allow us to handle x = (x,y) which is Proj. of FeatrePt in 2D Img plane: =(X/Z,Y/Z)=[(u-u0)/px,(v-v0)/py]
#include <visp3/vs/vpServo.h>						// implements Eeye-to-hand IBVS control law 
#include <visp3/vs/vpServoDisplay.h>
#include <stdio.h>
#include "CRSimulation.h"


void display_trajectory(const vpImage<unsigned char>& I, std::vector<vpPoint>& point, const vpHomogeneousMatrix& cMo,
	const vpCameraParameters& cam)
{
	static std::vector<vpImagePoint> traj[4];
	vpImagePoint cog;

	//Project the point at the given camera position

	for (unsigned int i = 0; i < 4; i++) {
		point[i].project(cMo);
		vpMeterPixelConversion::convertPoint(cam, point[i].get_x(), point[i].get_y(), cog);
		traj[i].push_back(cog);
	}
	for (unsigned int i = 0; i < 4; i++) {
		for (unsigned int j = 1; j < traj[i].size(); j++) {
			vpDisplay::displayLine(I, traj[i][j - 1], traj[i][j], vpColor::green);
		}
	}
}

int main()
{
	try {
		
		CRSimulation robot;
		robot.setSamplingTime(0.040);
		vpCameraParameters cam;  // Create a camera parameter container

		double px = 700; double py = 700; double u0 = 320; double v0 = 240; double zestimated = 0.24;
		cam.initPersProjWithoutDistortion(px, py, u0, v0); // Camera initialization with a perspective projection without distortion model

		vpHomogeneousMatrix eMc;
		vpTranslationVector _etc;
		vpRxyzVector _erc;

		_etc[0] = 0;
		_etc[1] = 0.22;
		_etc[3] = 0;
		_erc[0] = 0.5 * M_PI;
		_erc[1] = 0;
		_erc[2] = -0.5 * M_PI;//
		vpRotationMatrix eRc(_erc);
		eMc.buildFrom(_etc, eRc); //this matrix is subject to change in ETH config. as the robot moves

		//robot.getCameraParameters(cam, I);

		vpColVector q(3);
		vpColVector qd(3);
		
		/* define the des. and init. position of the 
		configuration parameters of the robot; number of robot joints = 3
		q[0]=twist,  q[1]=bending,  q[2]=translation */ 

		q[0] = vpMath::rad(0); 
		q[1] = vpMath::rad(0); 
		q[2] = 0;

		qd[0] = vpMath::rad(30);
		qd[1] = vpMath::rad(85);
		qd[2] = 0.01;

		// cMe: pose of end-effector of the robot w.r.t camera frame {c}
		vpHomogeneousMatrix cMe = eMc.inverse(); 
		// fMe: pose of end-effector of the robot w.r.t reference frame {f}
		vpHomogeneousMatrix fMe = robot.get_fMe(q);
		// fMc: pose of camera w.r.t reference frame {f}
		vpHomogeneousMatrix fMc = fMe * eMc;  

		/*wMc: position of the camera, wMo:the 
		position of the object in the world frame.*/
		vpHomogeneousMatrix fMed = robot.get_fMe(qd);
		vpHomogeneousMatrix edMc = fMed.inverse() * fMc;
		vpHomogeneousMatrix cMed = edMc.inverse();
     
		/* Transforms velocities from end-effector frame to
		camera farme. This matrix is also called velocity 
		twist transformation matrix.*/ 
		vpVelocityTwistMatrix cVe;  
		vpMatrix eJe;                

		/* Define 3D positions of feature points, which are assumed
		to be the conrners of a square marker fixed to the 
		end-effector frame, w.r.t the end-effector frame */ 
		std::vector<vpPoint> point(4);
		point[0] = vpPoint(-0.0035, 0, -0.0035);// meter
		point[1] = vpPoint(+0.0035, 0, -0.0035);
		point[2] = vpPoint(+0.0035, 0, +0.0035);
		point[3] = vpPoint(-0.0035, 0, +0.0035);

		vpServo task;
		task.setServo(vpServo::EYETOHAND_L_cVe_eJe);
		task.setInteractionMatrixType(vpServo::CURRENT);
		task.setLambda(0.2);
		task.set_cVe(cMe);     
		robot.get_eJe(q, eJe);  
		task.set_eJe(eJe);     

		vpFeaturePoint p[4], pd[4];
		
		for (unsigned int i = 0; i < 4; i++) {
			/* point[i].track(cMed): 
			convert from world coordinate
			in end-effector frame to normalized
			 x and y in image plane*/
			point[i].track(cMed); 
			/*vpFeatureBuilder::create(pd[i], point[i]): 
			pd[i].set_x (point[i].get_x());
			pd[i].set_y(point[i].get_y());
			p[i].set_Z(cMe[2][3]);*/
			vpFeatureBuilder::create(pd[i], point[i]); 
			point[i].track(cMe);
			vpFeatureBuilder::create(p[i], point[i]);
			task.addFeature(p[i], pd[i]); 
			/*void vpServo::addFeature(vpBasicFeature &s_cur,
			vpBasicFeature &s_star, unsigned int select)*/
		}

		vpImage<unsigned char> Iint(2*v0, 2*u0, 0);
		vpDisplayGDI displayInt(Iint, 0, 0, "Internal view");
			
		while (1)
		{
			fMe = robot.get_fMe(q);
			eMc = fMe.inverse() * fMc;
			cMe = eMc.inverse();
			for (unsigned int i = 0; i < 4; i++)
			{
				point[i].track(cMe);
				vpFeatureBuilder::create(p[i], point[i]);
			}	
			task.set_cVe(cMe);     
			robot.get_eJe(q, eJe); 
			task.set_eJe(eJe);     
			vpColVector qdot = task.computeControlLaw(); // IBVS Control law 
			robot.get_q(qdot, q);	                 		
						
			vpColVector error = task.computeError();
			double error_value = error.sumSquare();
			if (error_value < 1e-6)
			{
				std::cout <<"Control task is successfully completed!" << std::endl;
				//return 0;
			}
			vpDisplay::display(Iint); 
			display_trajectory(Iint, point, cMe, cam);
			vpServoDisplay::display(task, cam, Iint, vpColor::green, vpColor::red);
			vpDisplay::flush(Iint);
			}
		//Before exiting the program, we free all the memory by killing the task.
		task.kill();
	}
	catch (const vpException & e) {
		std::cout << "Catch an exception: " << e << std::endl;
		std::cin.get();
	}
}