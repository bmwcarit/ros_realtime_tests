set title "timer_tests plot PandaBoard 3.4.0-rt17+ PREEMPT RT no CPU load  -  <amount_measurements> samples  "
set xlabel "Latency in micro seconds - MIN:  <min>us  AVG: <avg>us MAX: <max>us"
set ylabel "Number of latency samples"
set yrange [0.7:]
set logscale y
set xrange [1:400]
set terminal jpeg size 1920,1080
set output "<Output_Name>.jpg"
plot "</path/to/logfile.log>" u 1:2 t 'Latency_Occurrence' w steps
