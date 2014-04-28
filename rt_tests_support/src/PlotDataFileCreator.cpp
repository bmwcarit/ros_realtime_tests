/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include <fstream>
#include <iomanip>
#include <rt_tests_support/PlotDataFileCreator.h>

PlotDataFileCreator::PlotDataFileCreator()
{
}

void PlotDataFileCreator::createPlottableDatafile(std::string filename, std::string preamble, MeasurementDataEvaluator* data, const int precisionDivisor)
{
	const int maxValue = data->getMaxValue()/precisionDivisor;
	const int minValue = data->getMinValue()/precisionDivisor;
	const int latHitArraySize = maxValue + 1;
	const long* plotValues = data->getData();
	int hits[latHitArraySize];
	for(int i = 0; i < latHitArraySize; i++)
	{
		hits[i] = 0;
	}

	const int loopLength = data->getDataSize();
	for(int i = 0; i < loopLength; i++)
	{
		if(plotValues[i] >= 0)
		{
			hits[plotValues[i]/precisionDivisor]++;
		}
	}

	std::ofstream fs;
	fs.open(filename.c_str());
	fs << preamble;
	for(int i = 0; i < latHitArraySize; i++)
	{
		fs << std::setfill('0') << std::setw(6) << i << " \t" << std::setfill('0') << std::setw(6) << hits[i] << std::endl;
	}

	if(minValue < 0)
	{
		const int negHitArraySize = minValue*(-1) + 1;
		fs << "# negative Values following" << std::endl;
		int negHits[negHitArraySize];
		for(int i = 0; i < negHitArraySize; i++)
		{
			negHits[i] = 0;
		}
		for(int i = 0; i < loopLength; i++)
		{
			if(plotValues[i] < 0)
			{
				negHits[(plotValues[i] * (-1))/precisionDivisor]++;
			}
		}

		for(int i = 1; i < negHitArraySize; i++)
		{
			fs << "#-" << std::setfill('0') << std::setw(6) << i << " \t" << std::setfill('0') << std::setw(6) << negHits[i] << std::endl;
		}
	}

	fs << std::endl << "end" << std::endl;
	fs.close();
}

PlotDataFileCreator::~PlotDataFileCreator()
{
}

