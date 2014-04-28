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

Subscriber::Subscriber(const std::string& topic, ros::NodeHandle* nodeHandle, int amountMessages) :
	lastSeq(messageMissing), outOfOrderCounter(0),
	latenciesNs((long*) malloc(sizeof(long) * amountMessages)),
	amountMessages(amountMessages), nodeHandle(nodeHandle),
	rosSubscriber(nodeHandle->subscribe(topic, 1000, &Subscriber::messageCallback, this))
{
	for(int i = 0; i < amountMessages; i++)
	{
		latenciesNs[i] = messageMissing;
	}
}

void Subscriber::startMeasurement()
{
	lastSeq = messageMissing;
	outOfOrderCounter = 0;
	ros::spin();
	measurementData = new MeasurementDataEvaluator(latenciesNs, amountMessages);
}

int Subscriber::getMinLatencyUs()
{
	return measurementData->getMinValue()/1000;
}

int Subscriber::getAvgLatencyUs()
{
	return measurementData->getAvgValue()/1000;
}

int Subscriber::getMaxLatencyUs()
{
	return measurementData->getMaxValue()/1000;
}

std::string Subscriber::getMeasurementSummary()
{
	std::stringstream ss;
	ss << "Amount messages: " << amountMessages << "; Messages out of order: " << getAmountMessagesOutOfOrder() << std::endl;
	ss << "MIN: " << getMinLatencyUs() << "us\tAVG: " << getAvgLatencyUs() << "us\tMAX: " << getMaxLatencyUs() << std::endl;
	if(measurementData->getMinValue() == messageMissing)
	{
		int messagesMissing = 0;
		ss << "Missing messages: |";
		for(int i = 0; i < amountMessages; i++)
		{
			if(latenciesNs[i] == messageMissing)
			{
				ss << i << "|";
				messagesMissing++;
			}
		}
		ss << std::endl << "Total of " << messagesMissing << " missing messages.";
		Logger::INFO(ss.str());
	}
	return ss.str();
}

void Subscriber::saveGnuplotData(std::string filename)
{
	std::stringstream measurementSummary(getMeasurementSummary());
	std::stringstream ss;
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
	plotter.createPlottableDatafile(filename, ss.str(), measurementData, 1000);
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
	latenciesNs[msg->seq] = (ts.tv_sec - msg->sec) * 1000000000 + (ts.tv_nsec - msg->nsec);
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
	delete latenciesNs;
}
