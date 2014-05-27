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
	topTenValues((valueWithIndex*) malloc(sizeof(valueWithIndex) * 10)),
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

valueWithIndex* MeasurementDataEvaluator::getTopTenValues()
{
	return topTenValues;
}

void MeasurementDataEvaluator::analyzeData()
{
	calcMinMaxAvg();
	findTopTenValues();
}

void MeasurementDataEvaluator::calcMinMaxAvg()
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

void MeasurementDataEvaluator::findTopTenValues()
{
	for(int i = 0; i < 10; i++)
	{
		topTenValues[i].value = minValue;
		topTenValues[i].index = 0;
	}
	for(int i = 0; i < dataSize; i++)
	{
		for(int j = 0; j < 10; j++)
		{
			if(data[i] > topTenValues[j].value)
			{
				for(int k = 9; k > j; k--)
				{
					topTenValues[k].value = topTenValues[k-1].value;
					topTenValues[k].index = topTenValues[k-1].index;
				}
				topTenValues[j].value = data[i];
				topTenValues[j].index = i;
				break;
			}
		}
	}
}

MeasurementDataEvaluator::~MeasurementDataEvaluator()
{
	delete data, topTenValues;
}
