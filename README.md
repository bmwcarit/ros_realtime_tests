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

* *--filePrefix &lt;file prefix&gt;*

	The string passed will be prepended to the log files that are generated once the measurement is complete.

* *--repetitions &lt;number of repetitions&gt;*

	Specify the number of measurement repetitions. If not specified, 1000 is chosen as default value.

* *--rtSched &lt;0/RR/FIFO&gt;*

	Specify which scheduling policy shall be used for the test node. You can choose between the three following policies:

	* 0: Run with default process priority and scheduling.
	* RR: Run with Real-Time process priority and Round-Robin scheduling.
	* FIFO: Run with Real-Time process priority and FIFO scheduling.

	If not specified, 0 is chosen as default value.

* *--timeout &lt;timeout in microseconds&gt;*

	Specify the timeout in microseconds with which the ROS-Timer is triggered. If not specified, 1000 (= 1 millisecond) is chosen as default value.

### Flags ###

* *--lowResource*

	If this flag is set, the main thread will use the `ros::spin()` function to trigger the call of the timeout callback function, instead of busy waiting with repeated calls to `ros::spinOnce()`. This reduces the generated CPU load drastically. However, the measured latencies are the accumulated sleep latencies occurring in the ROS-Timer function and in the sleep function, used in the implementation of `ros::spin()`.

---

## communication_tests Manual ##

### Synopsis ###

roslaunch communication_tests communication_tests.launch [Options]

### Options ###

* *filePrefix:=&lt;file prefix&gt;*

	The string passed will be prepended to the log files which are generated once the measurement is complete.

* *frequency:=&lt;publication frequency&gt;*

	Specify the frequency in Hertz with that the messages are published during the testing process. If not specified, 1000 is chosen as default value.

* *messages:=&lt;number of messages&gt;*

	Specify the number of messages to send during the measurement process. If not specified, 1000 is chosen as default value.

* *rtSched:=&lt;0/RR/FIFO&gt;*

	Specify which scheduling policy shall be used for the test nodes. You can choose between the three following policies:

	* 0: Run with default process priority and scheduling.
	* RR: Run with Real-Time process priority and Round-Robin scheduling.
	* FIFO: Run with Real-Time process priority and FIFO scheduling.

	If not specified, 0 is chosen as default value.
