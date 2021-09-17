Simulation code for the hexapod simulations of the paper

R. Der and G. Martius. Novel plasticity rule can explain the development of sensorimotor intelligence. Proceedings of the National Academy of Sciences, 112(45):E6224--E6232, 2015. arXiv Preprint http://arxiv.org/abs/1505.00835.

Check out the supplementary:
http://playfulmachines.com/DEP

First make sure you have lpzrobots installed, see README.txt in the top folder.

Compile the code in this directory via
make

then you get a program called "start"

You can specify the parameters in the commandline such that you can change the parameters as ou like. There are many switched, see
./start -h
for help.
There are also many parameters of the controller and of the simulation that you can change during the runtime and you can also specify at start with the -set "{key=value key2=value2...}" option. Please see the online help of lpzrobots (http://robot.informatik.uni-leipzig.de/software/doc/html/index.html#RunExample) to get an idea how it works and how to control the software.
There is also some useful resources in our book "The Playful Machine", see pdf on playfulmachines.com,  page 303ff.


The naming of the controller parameters differs a bit from the paper:
synboost := activity level = kappa in paper
urate    := update rate (speed of synaptic dynamics)
         tau (paper) =1/(simulation_rate*urate) = 1/(50* urate)


Video 3:
(A) walking with box-pile
./start  -walkmodel -tripod_neg -walkdelay -set "{synboost=2.2 urate=0.05 epsh=0.0 timedist=4 delay=8}" -m 5 -boxpile

(the parameters given in -set are actually the standard parameters, so you could omit it)

(B) with lateral and phaseshift coupling
./start  -walkmodel -lateral_neg -walkdelay -legdelay -set "{synboost=2.2 urate=0.03 epsh=0.0 timedist=4 delay=15}" -m 5 -boxpile
Then change delay on the console (Press Ctrl+C (in terminal) and then type delay=12 (or another value))

Video 4:
(B) from snapshots:
./start -m 5 -boxpile
then Ctrl+C and
load 101 controller/hexa_loco_snapshots_...
(use tab to complete filenames)
You can also store controllers in any simulation with store 101 filename. Then you can edit the files, for instance to set urate=0 in the parameter section at end of the file. In this way the controller remain as fixed if loaded.

Video 7: motor babbling to learn model.
./start -m 5 -babbling 50000

Video 8: learning rule comparison
You can change the learning rule with parameter 'learningrule': 0: DEP, 1: DHL, 2: HL,
So with DHL:
./start  -set "{synboost=2.2 urate=0.03 epsh=0.0 timedist=4 delay=8 learningrule=1}" -m 5
Before run the normal DEP version and store the controller at some point, such that you can load it into this one (make sure you edit the file to set the learning rule to the right value)

You can use -f 1 to record all the parameters etc. There are tools to process these files, such as selectcolumns.pl. For instance to see the times when a parameter delay was changed run
selectcolumns.pl "t" < Hexapod_0.log | grep -A 1 "# delay"
You can also use -g 1 to a visual tools to look at all control parameters online with gnuplot.
Use ./start --help to get an overview.
