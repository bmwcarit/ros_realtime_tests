## ROS REALTIME TESTS ##

This repository contains two test suites to validate the real-time capabilities of your ROS setup:

* **timer_tests**:

	This test suite validates the real-time characteristics of the ROS timer function by repeatedly setting the timer to a specified value and measuring the latency, by which the call of the callback function missed the actual timeout value.

* **communication_tests**:

	This test suite validates the real-time characteristics of the Publish/Subscribe communication mechanism by starting one or multiple publisher and subscriber nodes and measuring the message transmission latency that occurs between the message's publication and its reception at the subscriber node(s).

---

## timer_tests Manual ##

### Synopsis ###

./timer_tests [Options] [Flags]

### Options ###

* *--busy &lt;0/1&gt;*

	Specify the measurement mode:

	* 0: Default mode, using ROS::spin() in the main thread.
	* 1: Busy mode. If this mode is specified, the main thread busy waits until the timer thread calls the callback function.

	Note: Default mode tests the most common scenario, where a timer is set while another thread runs ROS::spin. Busy mode uses ROS::spinOnce() without sleep and therefore only measures the latency of the sleep call in the timer thread.

	If not specified, default mode is used.

* *--fp &lt;file prefix&gt;*

	The string passed will be prepended to the log files that are generated once the measurement is complete.

* *--rpts &lt;number of repetitions&gt;*

	Specify the number of measurement repetitions.

	If not specified, 1000 is chosen as default value.

* *--rtSched &lt;0/RR/FIFO&gt;*

	Specify which scheduling policy shall be used for the test node. You can choose between the three following policies:

	* 0: Run with default process priority and scheduling.
	* RR: Run with Real-Time process priority and Round-Robin scheduling.
	* FIFO: Run with Real-Time process priority and FIFO scheduling.

	If not specified, 0 is chosen as default value.

* *--to &lt;timeout in microseconds&gt;*

	Specify the timeout in microseconds with which the ROS-Timer is triggered.

	If not specified, 1000 (= 1 millisecond) is chosen as default value.

---

## communication_tests Manual ##

### Synopsis ###

roslaunch communication_tests communication_tests.launch [Options]

### Options ###

* *amt:=&lt;amount messages&gt;*

	Specify the number of messages to send during the measurement process. If not specified, 1000 is chosen as default value.

* *dly:=&lt;start delay&gt;*

	Specify the amount of seconds to wait before starting the message publication process. Increase this value if the subscriber doesn't get the first messages.

	If not specified, 1 second is chosen as default value.

* *fp:=&lt;file prefix&gt;*

	The string passed will be prepended to the log files which are generated once the measurement is complete.

* *freq:=&lt;publication frequency&gt;*

	Specify the frequency in Hertz with that the messages are published during the testing process.

	If not specified, 1000 is chosen as default value.

* *rtSched:=&lt;0/RR/FIFO&gt;*

	Specify which scheduling policy shall be used for the test nodes. You can choose between the three following policies:

	* 0: Run with default process priority and scheduling.
	* RR: Run with Real-Time process priority and Round-Robin scheduling.
	* FIFO: Run with Real-Time process priority and FIFO scheduling.

	If not specified, 0 is chosen as default value.
