/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "OneShotLatencyMeasurer.h"

#include "Config.h"
#include <sys/mman.h>
#include <sys/utsname.h>
#include <rt_tests_support/Logger.h>
#include <rt_tests_support/PlotDataFileCreator.h>

#define NANO_TO_MICRO_DIVISOR 1000
#define SEC_TO_NANOSEC_MULTIPLIER 1000000000

OneShotLatencyMeasurer::OneShotLatencyMeasurer(const int loopLength, long timeoutNanoSeconds, ros::NodeHandle* nodeHandle, bool lockMemory) :
		loopLength(loopLength),
		timeoutSeconds(((double)timeoutNanoSeconds)/SEC_TO_NANOSEC_MULTIPLIER),
		timeoutNanoseconds(timeoutSeconds * SEC_TO_NANOSEC_MULTIPLIER),
		nodeHandle(nodeHandle),
		callbackCalled(false),
		callbackTs(),
		loopCounter(0),
		lockMemory(lockMemory),
		latencyData(new MeasurementDataEvaluator(loopLength)),
		reportedLatencyData(new MeasurementDataEvaluator(loopLength)),
		differenceData(new MeasurementDataEvaluator(loopLength))
{
}

void OneShotLatencyMeasurer::measure()
{
	measureOneshotTimerLatencies();
	latencyData->analyzeData();
	reportedLatencyData->analyzeData();
	differenceData->analyzeData();
}

void OneShotLatencyMeasurer::measureOneshotTimerLatencies()
{
	if(lockMemory)
	{
		if(mlockall(MCL_CURRENT|MCL_FUTURE) != 0)
		{
			Logger::ERROR("Could'nt lock memory! Aborting...");
			exit(1);
		}
	}
	loopCounter = 0;
	ros::Timer rosTimer = nodeHandle->createTimer(ros::Duration(0.01), &OneShotLatencyMeasurer::timerCallback, this, true);
	spinUntilCallbackCalled();
	long latencyTempNs = 0;
	struct timespec startTs;

	for(loopCounter = 0; loopCounter < loopLength; loopCounter++)
	{
		rosTimer.setPeriod(ros::Duration(timeoutSeconds));
		clock_gettime(clock_id, &startTs);
		rosTimer.start();
		spinUntilCallbackCalled();
		latencyTempNs = ((callbackTs.tv_sec - startTs.tv_sec) * SEC_TO_NANOSEC_MULTIPLIER) + (callbackTs.tv_nsec - startTs.tv_nsec);
		latencyTempNs -= timeoutNanoseconds;
		latencyData->getData()[loopCounter] = latencyTempNs/NANO_TO_MICRO_DIVISOR;
		differenceData->getData()[loopCounter] = reportedLatencyData->getData()[loopCounter] - latencyData->getData()[loopCounter];
	}
	if(lockMemory)
	{
		munlockall();
	}
}

MeasurementDataEvaluator* OneShotLatencyMeasurer::getMeasuredLatencyData()
{
	return latencyData;
}

MeasurementDataEvaluator* OneShotLatencyMeasurer::getReportedLatencyData()
{
	return reportedLatencyData;
}

MeasurementDataEvaluator* OneShotLatencyMeasurer::getLatencyDifferenceData()
{
	return differenceData;
}

std::string OneShotLatencyMeasurer::getMeasurementSummary()
{
	std::stringstream ss;
	ss << "Measurement results with a loop length of " << loopLength << " and a timeout of " << (int) (timeoutNanoseconds/1000) << " us:" << std::endl;
	ss <<"Measured:\tMIN:  " << latencyData->getMinValue() << "us \tAVG:  " << latencyData->getAvgValue() << "us \tMAX:  " << latencyData->getMaxValue() << "us";
	ss << std::endl;
	ss <<"Reported:\tMIN:  " << reportedLatencyData->getMinValue() << "us \tAVG:  " << reportedLatencyData->getAvgValue() << "us \tMAX:  ";
	ss << reportedLatencyData->getMaxValue() << "us" << std::endl;
	ss << "Difference:\tMIN: " << differenceData->getMinValue() << "us\tAVG: " << differenceData->getAvgValue() << "us\tMAX: " << differenceData->getMaxValue() << "us";
	ss << std::endl;
	return ss.str();
}

void OneShotLatencyMeasurer::printMeasurementResults()
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

void OneShotLatencyMeasurer::saveMeasuredLatencyGnuplotData(std::string filename)
{
	saveGnuplotData(filename, latencyData);
}

void OneShotLatencyMeasurer::saveReportedLatencyGnuplotData(std::string filename)
{
	saveGnuplotData(filename, reportedLatencyData);
}
void OneShotLatencyMeasurer::saveDiffGnuplotData(std::string filename)
{
	saveGnuplotData(filename, differenceData);
}

void OneShotLatencyMeasurer::saveGnuplotData(std::string filename, MeasurementDataEvaluator* measurementData)
{
	struct utsname unameResponse;
	int rc = uname(&unameResponse);
	std::stringstream machineName;
	if(rc == 0)
	{
		machineName << unameResponse.nodename << " " << unameResponse.sysname << " " << unameResponse.release;
	}
	Config* config = Config::getConfig();
	std::stringstream ss;
	ss << "set title \"timer_tests plot " << machineName.str() << " -  " << loopLength << " samples  ";
	if(config->testnodeRT)
	{
		ss << "test node RT ";
		if(config->fifoScheduling)
		{
			ss << "FIFO";
		} else {
			ss << "RR";
		}
	}
	ss << "\"" << std::endl;
	ss << "set xlabel \"Latency in micro seconds - MIN:  ";
	ss << measurementData->getMinValue() << "us  AVG: " << measurementData->getAvgValue() << "us MAX: " << measurementData->getMaxValue() << "us\"" << std::endl;
	ss << "set ylabel \"Number of latency samples\"" << std::endl << "set yrange [0.7:]" << std::endl << "set logscale y" << std::endl;
	int xrange = measurementData->getMaxValue() + 50;
	if(measurementData->getMaxValue() < 400)
	{
		xrange = 400;
	}
	ss << "set xrange [1:" << xrange << "]" << std::endl << "set xtics add(500, 1000)" << std::endl;
	ss << "set terminal jpeg size 1920,1080" << std::endl;
	ss << "set output \"" << filename << ".jpg\"" << std::endl;
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
	plotter.createPlottableDatafile(filename+".log", ss.str(), measurementData);
}

void OneShotLatencyMeasurer::timerCallback(const ros::TimerEvent& te)
{
	clock_gettime(clock_id, &callbackTs);
	long latencyReportedTemp = ((te.current_real.sec - te.current_expected.sec) * SEC_TO_NANOSEC_MULTIPLIER) + (te.current_real.nsec - te.current_expected.nsec);
	reportedLatencyData->getData()[loopCounter] = latencyReportedTemp/NANO_TO_MICRO_DIVISOR;
	callbackCalled = true;
}

void OneShotLatencyMeasurer::spinUntilCallbackCalled()
{
	callbackCalled = false;
	while(!callbackCalled && ros::ok())
	{
		ros::spinOnce();
	}
}

OneShotLatencyMeasurer::~OneShotLatencyMeasurer()
{
	delete latencyData, reportedLatencyData, differenceData;
}

