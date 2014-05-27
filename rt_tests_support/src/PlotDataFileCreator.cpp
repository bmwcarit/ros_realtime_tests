/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include <fstream>
#include <sstream>
#include <iomanip>
#include <rt_tests_support/PlotDataFileCreator.h>

PlotDataFileCreator::PlotDataFileCreator()
{
}

void PlotDataFileCreator::createPlottableDatafile(std::string filename, std::string preamble, MeasurementDataEvaluator* data, bool commentNeg)
{
	createPlottableDatafile(filename, preamble, data->getMaxValue(), data->getMinValue(), data->getData(), data->getDataSize(), commentNeg);
}

void PlotDataFileCreator::createPlottableDatafile(std::string filename, std::string preamble, int max, int min, long* data, int dataSize, bool commentNeg)
{
	const int maxValue = max;
	const int minValue = min;
	const int latHitArraySize = maxValue + 1;
	const long* plotValues = data;
	int hits[latHitArraySize];
	for(int i = 0; i < latHitArraySize; i++)
	{
		hits[i] = 0;
	}

	const int loopLength = dataSize;
	for(int i = 0; i < loopLength; i++)
	{
		if(plotValues[i] >= 0)
		{
			hits[plotValues[i]]++;
		}
	}

	std::ofstream fs;
	fs.open(filename.c_str());
	fs << preamble;
	std::stringstream positiveValues;
	for(int i = 0; i < latHitArraySize; i++)
	{
		positiveValues << std::setfill('0') << std::setw(6) << i << " \t" << std::setfill('0') << std::setw(6) << hits[i] << std::endl;
	}

	std::stringstream negativeValues("");
	if(minValue < 0)
	{
		const int negHitArraySize = minValue*(-1) + 1;
		if(commentNeg)
		{
			negativeValues << "# negative Values following" << std::endl;
		}
		int negHits[negHitArraySize];
		for(int i = 0; i < negHitArraySize; i++)
		{
			negHits[i] = 0;
		}
		for(int i = 0; i < loopLength; i++)
		{
			if(plotValues[i] < 0)
			{
				negHits[plotValues[i] * (-1)]++;
			}
		}

		for(int i = negHitArraySize-1; i > 0; i--)
		{
			if(commentNeg)
			{
				negativeValues << "#";
			}
			negativeValues << "-" << std::setfill('0') << std::setw(6) << i << " \t" << std::setfill('0') << std::setw(6) << negHits[i] << std::endl;
		}
	}
	if(commentNeg)
	{
		fs << positiveValues.str();
		fs << negativeValues.str();
	} else {
		fs << negativeValues.str();
		fs << positiveValues.str();
	}
	fs << std::endl << "end" << std::endl;
	fs.close();
}

PlotDataFileCreator::~PlotDataFileCreator()
{
}

