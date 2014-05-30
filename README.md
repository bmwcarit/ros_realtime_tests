## ROS REALTIME TESTS ##

This repository contains three test suites to validate the real-time capabilities of your ROS setup:

* **cyclic_timer_tests**:

	This test suite validates the real-time characteristics of the cyclic ROS Timer function
by setting the timer to a specific frequency and measuring the jitter by which the calls to the
callback function miss the specified frequency.

* **oneshot_timer_tests**:

	This test suite validates the real-time characteristics of the ROS OneShot Timer function
by repeatedly setting the timer to a specified value and measuring the latency, by which the call
of the callback function missed the actual timeout value.

* **communication_tests**:

	This test suite validates the real-time characteristics of the Publish/Subscribe
communication mechanism by starting one or multiple publisher and subscriber nodes and measuring
the message transmission latency that occurs between the message's publication and its reception
at the subscriber node(s).

Generating plots to visualize your test results can be done by simply invoking gnuplot with the
path to the corresponding logfile:
```sh
gnuplot [results.log]
```

---

## cyclic_timer_tests Manual ##

### Synopsis ###
```sh
./cyclic_timer_tests [Options]
```

### Options ###

* *-f &lt;frequency&gt;*

	Specify the frequency in Hertz with that the timer shall call the callback function during
the testing process.

	If not specified, 1000 is chosen as default value.

* *-p &lt;file name prefix&gt;*

	The string passed will be prepended to the names of the log files that are generated once
the measurement is complete.

* *-r &lt;number of repetitions&gt;*

	Specify the number of measurement repetitions.

	If not specified, 1000 is chosen as default value.

* *-s &lt;0/RR/FIFO&gt;*

	Specify which scheduling policy shall be used for the test node. You can choose between
the three following policies:

	* 0: Run with default process priority and scheduling.
	* RR: Run with Real-Time process priority and Round-Robin scheduling.
	* FIFO: Run with Real-Time process priority and FIFO scheduling.

	If not specified, 0 is chosen as default value.

---

## oneshot_timer_tests Manual ##

### Synopsis ###
```sh
./oneshot_timer_tests [Options]
```

### Options ###

* *-b &lt;0/1&gt;*

	Specify the measurement mode:

	* 0: Default mode, using ROS::spin() in the main thread.
	* 1: Busy mode. If this mode is specified, the main thread busy waits until the timer
thread calls the callback function.

	Note: Default mode tests the most common scenario, where a timer is set while another
thread runs ROS::spin. Busy mode uses ROS::spinOnce() without sleep and therefore only measures
the latency of the sleep call in the timer thread.

	If not specified, default mode is used.

* *-p &lt;file name prefix&gt;*

	The string passed will be prepended to the names of the log files that are generated once
the measurement is complete.

* *-r &lt;number of repetitions&gt;*

	Specify the number of measurement repetitions.

	If not specified, 1000 is chosen as default value.

* *-s &lt;0/RR/FIFO&gt;*

	Specify which scheduling policy shall be used for the test node. You can choose between
the three following policies:

	* 0: Run with default process priority and scheduling.
	* RR: Run with Real-Time process priority and Round-Robin scheduling.
	* FIFO: Run with Real-Time process priority and FIFO scheduling.

	If not specified, 0 is chosen as default value.

* *-t &lt;timeout in microseconds&gt;*

	Specify the timeout in microseconds with which the ROS-Timer is triggered.

	If not specified, 1000 (= 1 millisecond) is chosen as default value.

---

## communication_tests Manual ##

### Synopsis ###
```sh
roslaunch communication_tests communication_tests.launch [Options]
```

### Options ###

* *d:=&lt;start delay&gt;*

	Specify the amount of seconds to wait before starting the message publication process.
Increase this value if the subscriber doesn't get the first messages.

	If not specified, 1 second is chosen as default value.

* *f:=&lt;publication frequency&gt;*

	Specify the frequency in Hertz with that the messages are published during the
testing process.

	If not specified, 1000 is chosen as default value.

* *l:=&lt;message payload length&gt;*

	Specify the size of the additional message payload in Bytes. If a value >0 is specified,
a payload of the specified size is appended to each of the test messages.

	If not specified, 0 is chosen as default value.

* *p:=&lt;file name prefix&gt;*

	The string passed will be prepended to the names of the log files that are generated once
the measurement is complete.

* *r:=&lt;number of repetitions&gt;*

	Specify the amount of messages to send during the measurement process. If not specified,
1000 is chosen as default value.

* *s:=&lt;0/RR/FIFO&gt;*

	Specify which scheduling policy shall be used for the test nodes. You can choose between
the three following policies:

	* 0: Run with default process priority and scheduling.
	* RR: Run with Real-Time process priority and Round-Robin scheduling.
	* FIFO: Run with Real-Time process priority and FIFO scheduling.

	If not specified, 0 is chosen as default value.
