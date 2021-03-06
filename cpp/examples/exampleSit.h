/*
 * exampleSit.h
 *
 *  Created on: 11 Oct 2018
 *      Author: kleinwrt
 */

/** \file
 *  Definitions for exampleSit.
 *
 *  \author Claus Kleinwort, DESY, 2018 (Claus.Kleinwort@desy.de)
 *
 *  \copyright
 *  Copyright (c) 2018 Deutsches Elektronen-Synchroton,
 *  Member of the Helmholtz Association, (DESY), HAMBURG, GERMANY \n\n
 *  This library is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Library General Public License as
 *  published by the Free Software Foundation; either version 2 of the
 *  License, or (at your option) any later version. \n\n
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details. \n\n
 *  You should have received a copy of the GNU Library General Public
 *  License along with this program (see the file COPYING.LIB for more
 *  details); if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef EXAMPLESIT_H_
#define EXAMPLESIT_H_

#include "exampleUtil.h"

//! Namespace for the general broken lines package
namespace gbl {

GblDetectorLayer CreateLayerSit(const std::string aName, unsigned int layer,
		double xPos, double yPos, double zPos, double thickness, double uAngle,
		double uRes); // 1D measurement
GblDetectorLayer CreateLayerSit(const std::string aName, unsigned int layer,
		double xPos, double yPos, double zPos, double thickness, double uAngle,
		double uRes, double vAngle, double vRes); // 2D measurement

}

#endif /* EXAMPLESIT_H_ */
