/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#ifndef MEASUREMENTDATAEVALUATOR_H_
#define MEASUREMENTDATAEVALUATOR_H_

class MeasurementDataEvaluator {
public:
	struct latencyWithIndex {
		long latency;
		int index;
	};

	MeasurementDataEvaluator(int dataSize);
	long getMinValue();
	long getAvgValue();
	long getMaxValue();
	long* getData();
	int getDataSize();
	long* getTopTenLatencies();
	int* getTopTenLatencyIndices();
	void analyzeData();
	~MeasurementDataEvaluator();
private:
	long* data;
	int dataSize;
	long* topTenLatencies;
	int* topTenLatencyIndices;
	long minValue;
	long maxValue;
	long long avgValue;

	MeasurementDataEvaluator();
};

#endif //MEASUREMENTDATAEVALUATOR_H_
