/****************************************************************************
*
* $Id: vpViper.cpp 4206 2013-04-13 07:29:06Z fspindle $
*
* This file is part of the ViSP software.
* Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
*
* This software is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* ("GPL") version 2 as published by the Free Software Foundation.
* See the file LICENSE.txt at the root directory of this source
* distribution for additional information about the GNU GPL.
*
* For using ViSP with software that can not be combined with the GNU
* GPL, please contact INRIA about acquiring a ViSP Professional
* Edition License.
*
* See http://www.irisa.fr/lagadic/visp/visp.html for more information.
*
* This software was developed at:
* INRIA Rennes - Bretagne Atlantique
* Campus Universitaire de Beaulieu
* 35042 Rennes Cedex
* France
* http://www.irisa.fr/lagadic
*
* If you have questions regarding the use of this file, please contact
* INRIA at visp@inria.fr
*
* This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
* WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*
*
* Description:
* Interface for a  generic ADEPT Viper (either 650 or 850) robot.
*
* Authors:
* Fabien Spindler
*
*****************************************************************************/

/*!
\file vpViper.cpp
Modelisation of the ADEPT Viper 650 or 850 robot.
*/



#include <visp/vpDebug.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpRobotException.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpViper.h>
#include <cmath>    // std::fabs
#include <limits>   // numeric_limits
#include "vpViper.h"

const unsigned int vpViper::njoint = 6;

/*!
Default constructor.
*/
vpViper::vpViper()
{
	// Default values are initialized

	// Denavit Hartenberg parameters
	a1 = 0.075;
	a2 = 0.365;
	a3 = 0.090;
	d1 = 0.335;
	d4 = 0.405;
	d6 = 0.080;
	c56 = -341.33 / 9102.22;

	// Software joint limits in radians
	joint_min.resize(njoint);
	joint_min[0] = vpMath::rad(-170);
	joint_min[1] = vpMath::rad(-190);
	joint_min[2] = vpMath::rad(-29);
	joint_min[3] = vpMath::rad(-190);
	joint_min[4] = vpMath::rad(-120);
	joint_min[5] = vpMath::rad(-360);
	joint_max.resize(njoint);
	joint_max[0] = vpMath::rad(170);
	joint_max[1] = vpMath::rad(45);
	joint_max[2] = vpMath::rad(256);
	joint_max[3] = vpMath::rad(190);
	joint_max[4] = vpMath::rad(120);
	joint_max[5] = vpMath::rad(360);

	// End effector to camera transformation
	eMc.setIdentity();
}

