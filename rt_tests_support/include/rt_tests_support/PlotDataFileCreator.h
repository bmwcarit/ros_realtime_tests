/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef PLOTDATAFILECREATOR_H_
#define PLOTDATAFILECREATOR_H_

#include <string>
#include <rt_tests_support/MeasurementDataEvaluator.h>

class PlotDataFileCreator {
public:
	PlotDataFileCreator();
	void createPlottableDatafile(std::string filename, std::string preamble, MeasurementDataEvaluator* data);
	~PlotDataFileCreator();
};

#endif //PLOTDATAFILECREATOR_H_
