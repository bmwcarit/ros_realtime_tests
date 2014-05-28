/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "CyclicLatencyMeasurer.h"

#include <sys/mman.h>
#include <rt_tests_support/Logger.h>
#include <rt_tests_support/PlotDataFileCreator.h>

#define NANO_TO_MICRO_DIVISOR 1000
#define SEC_TO_MICROSEC_MULTIPLIER 1000000
#define SEC_TO_NANOSEC_MULTIPLIER 1000000000

CyclicLatencyMeasurer::CyclicLatencyMeasurer() :
		config(Config::getConfig()),
		callbackCounter(0),
		jitter(new MeasurementDataEvaluator(config->loops)),
		callbackTimestamps((struct timespec*) malloc(sizeof(struct timespec) * (config->loops+1)))
{
}

void CyclicLatencyMeasurer::measure()
{
	if(config->testnodeRT)
	{
		if(mlockall(MCL_CURRENT|MCL_FUTURE) != 0)
		{
			Logger::ERROR("Couldn't lock memory! Aborting...");
			exit(1);
		}
	}
	measureTimerJitter();
	if(config->testnodeRT)
	{
		munlockall();
	}
	jitter->analyzeData();
	printMeasurementResults();
	savePlottableData();
}

void CyclicLatencyMeasurer::measureTimerJitter()
{
	callbackCounter = 0;
	const long expectedTimeout = (int) (SEC_TO_MICROSEC_MULTIPLIER/config->frequency);
	ros::Timer rosTimer = config->nodeHandle->createTimer(ros::Duration(1.0/(double) config->frequency), &CyclicLatencyMeasurer::timerCallback, this);
	ros::spin();
	for(int i = 0; i < config->loops; i++)
	{
		long long temp = (callbackTimestamps[i+1].tv_sec - callbackTimestamps[i].tv_sec) * SEC_TO_NANOSEC_MULTIPLIER;
		temp += callbackTimestamps[i+1].tv_nsec - callbackTimestamps[i].tv_nsec;
		temp /= NANO_TO_MICRO_DIVISOR;
		temp -= expectedTimeout;
		jitter->getData()[i] = (long) temp;
	}
}

std::string CyclicLatencyMeasurer::getMeasurementSummary()
{
	std::stringstream ss;
	ss << "Measurement results with a total of " << config->loops << " measurements and a frequency of " << config->frequency << "Hz:" << std::endl;
	ss << "Jitter:\tMIN: " << jitter->getMinValue() << "us\tAVG: " << jitter->getAvgValue() << "us\tMAX: " << jitter->getMaxValue() << "us";
	ss << std::endl << jitter->getBoundaryValueSummary();
	return ss.str();
}

void CyclicLatencyMeasurer::printMeasurementResults()
{
	std::stringstream measurementSummary(getMeasurementSummary());
	while(!measurementSummary.eof() && !measurementSummary.fail())
	{
		char line[512];
		measurementSummary.getline(line, 512);
		if(line[0] != '\0')
		{
			Logger::INFO(line);
		}
	}
}

void CyclicLatencyMeasurer::savePlottableData()
{
	std::stringstream ss;
	ss << "set title \"" << config->getTitle() << "\"" << std::endl;
	ss << "set xlabel \"Jitter in micro seconds - MIN:  " << jitter->getMinValue();
	ss <<  "us  AVG: " << jitter->getAvgValue() <<  "us MAX: " << jitter->getMaxValue() <<  "us\"" << std::endl;
	ss << "set ylabel \"Number of samples\"" << std::endl << "set yrange [0.7:]" << std::endl << "set logscale y" << std::endl;
	int xrange;
	if(jitter->getMaxValue() < 200 && jitter->getMinValue() > -200)
	{
		xrange = 200;
	} else {
		xrange = (jitter->getMaxValue() < (jitter->getMinValue()*-1))?(jitter->getMinValue()*-1):jitter->getMaxValue(); // xrange = MAX(maxVal, minVal*-1)
		xrange += 50;
	}
	ss << "set xrange [" << -xrange << ":" << xrange << "]" << std::endl << "set xtics add(500, 1000)" << std::endl;
	ss << "set terminal jpeg size 1920,1080" << std::endl;
	ss << "set output \"" << config->getFilename() << ".jpg\"" << std::endl;
	ss << "plot \"-\" u 1:2 t 'Latency_Occurrence' w steps" << std::endl;
	ss << "# Plot data for gnuplot" << std::endl;
	std::stringstream measurementSummary(getMeasurementSummary());
	while(!measurementSummary.eof() && !measurementSummary.fail())
	{
		char line[512];
		measurementSummary.getline(line, 512);
		if(line[0] != '\0')
		{
			ss << "# " << line << std::endl;
		}
	}
	PlotDataFileCreator plotter;
	plotter.createPlottableDatafile(config->getFilename()+".log", ss.str(), jitter, false);
}

void CyclicLatencyMeasurer::timerCallback(const ros::TimerEvent& te)
{
	clock_gettime(clock_id, &callbackTimestamps[callbackCounter]);
	callbackCounter++;
	if(callbackCounter > config->loops)
	{
		ros::shutdown();
	}
}

CyclicLatencyMeasurer::~CyclicLatencyMeasurer()
{
	delete callbackTimestamps;
}
