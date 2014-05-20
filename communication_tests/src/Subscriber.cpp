/**
* Copyright (C) 2014, BMW Car IT GmbH
* Author: Jonas Sticha (Jonas.Sticha@bmw-carit.de)
*
* This software is licensed under BSD 3-clause License
* (see http://spdx.org/licenses/BSD-3-Clause).
**/

#include "Subscriber.h"
#include <sstream>
#include <rt_tests_support/Logger.h>
#include <rt_tests_support/PlotDataFileCreator.h>

#define NANO_TO_MICRO_DIVISOR 1000

Subscriber::Subscriber(const std::string& topic) :
	config(Config::getConfig()),
	lastSeq(messageMissing), outOfOrderCounter(0), amountMessages(config->amountMessages),
	rosSubscriber(config->nodeHandle->subscribe(topic, 1000, &Subscriber::messageCallback, this)),
	measurementData(new MeasurementDataEvaluator(config->amountMessages))
{
}

void Subscriber::startMeasurement()
{
	lastSeq = messageMissing;
	for(int i = 0; i < amountMessages; i++)
	{
		measurementData->getData()[i] = messageMissing;
	}
	outOfOrderCounter = 0;
	ros::spin();
	measurementData->analyzeData();
}

std::string Subscriber::getMeasurementSummary()
{
	std::stringstream ss;
	ss << "Amount messages: " << config->amountMessages << "; Messages out of order: " << getAmountMessagesOutOfOrder() << std::endl;
	ss << "MIN: " << measurementData->getMinValue() << "us\tAVG: " << measurementData->getAvgValue() << "us\tMAX: " << measurementData->getMaxValue() << "us" << std::endl;
	ss << "Indices of top values(|latency:index|): |";
	for(int i = 0; i < 10; i++)
	{
		ss << measurementData->getTopTenLatencies()[i] << ":" << measurementData->getTopTenLatencyIndices()[i] << "|";
	}
	ss << std::endl;
	if(measurementData->getMinValue() == messageMissing)
	{
		int messagesMissing = 0;
		ss << "Missing messages: |";
		for(int i = 0; i < config->amountMessages; i++)
		{
			if(measurementData->getData()[i] == messageMissing)
			{
				ss << i << "|";
				messagesMissing++;
			}
		}
		ss << std::endl << "Total of " << messagesMissing << " missing messages." << std::endl;
	}
	return ss.str();
}

void Subscriber::saveGnuplotData()
{
	std::string filename = config->getFilename();
	std::stringstream measurementSummary(getMeasurementSummary());
	std::stringstream ss;
	ss << "set title \"" << config->getTitle() << "\"" << std::endl;
	ss << "set xlabel \"Latency in micro seconds - MIN:  ";
	ss << measurementData->getMinValue() << "us  AVG: " << measurementData->getAvgValue() << "us MAX: " << measurementData->getMaxValue() << "us\"" << std::endl;
	ss << "set ylabel \"Number of latency samples\"" << std::endl << "set yrange [0.7:]" << std::endl << "set logscale y" << std::endl;
	int xrange = measurementData->getMaxValue() + 50;
	ss << "set xrange [1:" << xrange << "]" << std::endl << "set xtics add(500, 1000)" << std::endl;
	ss << "set terminal jpeg size 1920,1080" << std::endl;
	ss << "set output \"" << filename << ".jpg\"" << std::endl;
	ss << "plot \"-\" u 1:2 t 'Latency_Occurrence' w steps" << std::endl;
	ss << "# Plot data for gnuplot" << std::endl;
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

void Subscriber::printMeasurementResults()
{
	Logger::INFO("Measurement results:");
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

MeasurementDataEvaluator* Subscriber::getMeasurementData()
{
	return measurementData;
}

int Subscriber::getAmountMessagesOutOfOrder()
{
	return outOfOrderCounter;
}

void Subscriber::messageCallback(const communication_tests::timestamp_msg::ConstPtr& msg)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	measurementData->getData()[msg->seq] = ((ts.tv_sec - msg->sec) * 1000000000 + (ts.tv_nsec - msg->nsec))/NANO_TO_MICRO_DIVISOR;
	if((int) msg->seq < lastSeq)
	{
		outOfOrderCounter++;
	}
	lastSeq = msg->seq;
	if(msg->last_msg)
	{
		ros::shutdown();
	}
}

Subscriber::~Subscriber()
{
	delete measurementData;
}
