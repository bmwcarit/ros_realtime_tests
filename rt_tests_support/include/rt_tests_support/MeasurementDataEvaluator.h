/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef MEASUREMENTDATAEVALUATOR_H_
#define MEASUREMENTDATAEVALUATOR_H_

#include <string.h>

typedef struct {
	long value;
	int index;
} valueWithIndex;

class MeasurementDataEvaluator {
public:
	MeasurementDataEvaluator(int dataSize);
	long getMinValue();
	long getAvgValue();
	long getMaxValue();
	long* getData();
	int getDataSize();
	valueWithIndex* getTopTenValues();
	valueWithIndex* getLowestTenValues();
	std::string getBoundaryValueSummary();
	void analyzeData();
	~MeasurementDataEvaluator();
private:
	long* data;
	int dataSize;
	valueWithIndex* topTenValues; // Big -> small
	valueWithIndex* lowestTenValues; // Small -> big
	long minValue;
	long maxValue;
	long long avgValue;

	void calcMinMaxAvg();
	void findTopTenValues();
	void findLowestTenValues();
	MeasurementDataEvaluator();
};

#endif //MEASUREMENTDATAEVALUATOR_H_
