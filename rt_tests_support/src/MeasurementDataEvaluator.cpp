/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include <stdlib.h>
#include <rt_tests_support/MeasurementDataEvaluator.h>

MeasurementDataEvaluator::MeasurementDataEvaluator(int dataSize) :
	data((long*) malloc(sizeof(long) * dataSize)), dataSize(dataSize),
	minValue(0), maxValue(0), avgValue(0)
{
	for(int i = 0; i < dataSize; i++)
	{
		data[i] = 0;
	}
}

long MeasurementDataEvaluator::getMinValue()
{
	return minValue;
}

long MeasurementDataEvaluator::getAvgValue()
{
	return avgValue;
}

long MeasurementDataEvaluator::getMaxValue()
{
	return maxValue;
}

long* MeasurementDataEvaluator::getData()
{
	return data;
}

int MeasurementDataEvaluator::getDataSize()
{
	return dataSize;
}

void MeasurementDataEvaluator::analyzeData()
{
	maxValue = data[0];
	minValue = data[0];
	avgValue = 0.0;
	for(int i = 0; i < dataSize; i++)
	{
		if(data[i] > maxValue)
		{
			maxValue = data[i];
		}
		if(data[i] < minValue)
		{
			minValue = data[i];
		}
		avgValue += data[i];
	}
	avgValue /= dataSize;
}

MeasurementDataEvaluator::~MeasurementDataEvaluator()
{
	delete data;
}
