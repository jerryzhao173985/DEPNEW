/***************************************************************************
 *   Copyright (C) 2015 by Robot Group Leipzig                             *
 *    georg.martius@web.de                                                 *
 *    ralfder@mis.mpg.de                                                   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 *
 ***************************************************************************/

#include <iostream>
#include <fstream>


#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/operators.h>
#include <ode_robots/boxpile.h>
#include <ode_robots/sensor.h>
#include <ode_robots/speedsensor.h>
//#include <ode_robots/contactsensor.h>
//#include <ode_robots/relativepositionsensor.h>

#include <selforg/switchcontroller.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/motorbabbler.h>
#include <selforg/stl_adds.h>

#include <ode_robots/hexapod.h>
#include "dep.h"


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace matrix;
bool track = false;
const char* trackfilename=0;
const char* loadcontroller=0;
int numwalls=0;
bool useSine=false;
bool useSwitch=false;
bool useDEP=true;
bool fix=false;
double noisecontrol=0;
bool whitenoise=false;
bool walkmodel=false;
bool tripod=false;
bool tripod_neg=false;
bool lateral_neg=false;
bool walkdelay=false;
bool legdelay=false;
bool boxpile=false;
int babbling=0;
double initHeight=1.2;
bool randomctrl=false;

const char* config_name = "config.txt";

double realtimefactor; //changeable speed for simulation graphics for recording images


/// Class to wrap a sensor and feed its delayed values.
class DelaySensor : public Sensor, public Configurable {
public:
  /// the sensor is supposed to be initialized (or gets initialized elsewhere before sense etc. is called)
  DelaySensor (std::shared_ptr<Sensor> src, int delay=1)
    : src(src), buffer(100), delay(delay), t(0) {
    assert(src);
    addParameter("delay", &this->delay, 0, 100, "delay in steps of sensor value");
    SensorMotorInfo smi=src->getBaseInfo();
    setBaseInfo(smi.changename("Delayed " + smi.name));
    setNamingFunc(src->getNamingFunc());
  }

  void init(Primitive* own, Joint* joint = 0) {
    // src->init(own, joint); // we do not want to init the sensor again. // debateable
  }

  bool sense(const GlobalData& globaldata){
    bool res =  src->sense(globaldata);
    buffer[t]=src->getList();
    t++;
    return res;
  }

  int getSensorNumber() const {
    return src->getSensorNumber();
  }

  std::list<sensor> getList() const {
    return buffer[t-1-delay];
  }

protected:
  std::shared_ptr<Sensor> src;
  RingBuffer<std::list<sensor> > buffer;
  int delay;
  long t;
};


class ThisSim : public Simulation {
public:
  StatisticTools stats;

  AbstractController* controller;
  OdeRobot* robot;

  ThisSim(){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    addColorAliasFile("colors.txt");
    setGroundTexture("Images/whiteground.jpg");

    setCaption("");  // if not set caption, it'll go to the default Lpzrobot caption
    // setCaption("DEP - Differential Extrinsic Plasticity (Der & Martius)");
    //setCaption ("Simulator by Martius et al");

  }


  // notice here pcc should pass (to function) by reference, or the DiamondConf wouldn't update
  void loadParams(DEP* dep, const char* filename = "config.txt"){
    //read from config file first || there must be "=" in the config file!!
    // std::ifstream is RAII, i.e. no need to call close
    std::ifstream cFile (filename);  //"config.txt"
    if (cFile.is_open())
    {
      std::string line;
      while(std::getline(cFile, line)){
        line.erase(std::remove_if(line.begin(), line.end(), ::isspace),line.end());
        if( line.empty() || line[0] == '#' )
        {
            continue;
        }
        auto delimiterPos = line.find("=");
        auto name = line.substr(0, delimiterPos);
        auto value = line.substr(delimiterPos + 1);
        
        if(name=="epsM"){ 
          dep->setParam("epsM", (double) std::stod(value));
        }else if(name=="epsh"){
          dep->setParam("epsh", (double) std::stod(value));
        }else if(name=="synboost"){
          dep->setParam("synboost", (double) std::stod(value)); 
        }else if(name=="urate"){
          dep->setParam("urate", (double) std::stod(value));
        }else if(name== "indnorm"){
          dep->setParam("indnorm", (int) std::stoi(value));
        }else if(name=="timedist"){
          dep->setParam("timedist", (int) std::stoi(value));
        }else if(name=="learningrule"){
          dep->setParam("learningrule", (int) std::stoi(value));
        }else if(name=="Time"){
          dep->setParam("Time", (int) std::stoi(value));
        }else{
          std::cout<< "some thing in the file cannot be assigned to the simulation controller." <<std::endl;
        }
        std::cout << name << " " << value << '\n';
      }
    }
    else 
    {
      std::cerr << "Couldn't open config file for reading.\n";
    }

  }



  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos// (Pos(-6.32561, 5.12705, 3.17278),  Pos(-130.771, -17.7744, 0));
    (Pos(1.6, 4.9945, 3.55472),  Pos(160.39, -25.768, 0));
    setCameraMode(Follow);

    global.odeConfig.setParam("noise", 0.0);
    global.odeConfig.setParam("controlinterval", 2);
    global.odeConfig.setParam("cameraspeed", 100);
    global.odeConfig.setParam("gravity", -9.81);
    global.odeConfig.setParam("realtimefactor", 0);
    setParam("UseQMPThread", false);

    if(boxpile){
      Boxpile* boxpile = new Boxpile(odeHandle, osgHandle,osg::Vec3(20.0, 20.0, 1.0), 100, 1,
                                     osg::Vec3(1, 1, 0.07), osg::Vec3(.4, .4, 0.02));
      boxpile->setColor("wall");
      boxpile->setPose(ROTM(M_PI/5.0,0,0,1)*TRANSM(20 ,0 , 0));
      global.obstacles.push_back(boxpile);
    }

    // stacked Playgrounds
    double scale = 1;
    double heightoffset = 0.1;
    double height = 0;
    for (int i=0; i< numwalls; i++){
      auto playground = new Playground(odeHandle, osgHandle,
                                  osg::Vec3((4+4*i)*scale, .2, heightoffset +i*height), 1, false);
      //      playground->setColor(Color((i & 1) == 0,(i & 2) == 0,(i & 3) == 0,0.3f));
      // playground->setTexture("Images/really_white.rgb");
      playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
      global.obstacles.push_back(playground);
    }


    controller=0;
    global.configs.push_back(this);
    /*******  H E X A P O D  *********/

    int numhexapods = 1;
    for ( int ii = 0; ii< numhexapods; ii++){

      HexapodConf myHexapodConf        = Hexapod::getDefaultConf();
      myHexapodConf.useTebiaMotors     = true;
      myHexapodConf.coxaPower          = 3.0;
      myHexapodConf.coxaSpeed          = 25;
      myHexapodConf.tebiaPower         = 1.8;
      myHexapodConf.coxaJointLimitV    = 0.5; // angle range for vertical dir. of legs
      myHexapodConf.coxaJointLimitH    = 0.8; // angle range for horizontal dir. of legs
      myHexapodConf.tebiaJointLimit    = 0.2;
      myHexapodConf.tebiaOffset        = 0;
      myHexapodConf.percentageBodyMass = .1;
      myHexapodConf.useBigBox          = false;
      myHexapodConf.tarsus             = true;
      myHexapodConf.numTarsusSections  = 1;
      myHexapodConf.useTarsusJoints    = true;
      myHexapodConf.legSpreading       = M_PI/10.0;

      OdeHandle rodeHandle = odeHandle;
      rodeHandle.substance.toRubber(20.); // sticky feet
      OdeRobot* hexapod = new Hexapod(rodeHandle, osgHandle.changeColor("Green"),
                                      myHexapodConf, "Hexapod_" + std::itos(ii));

      robot = hexapod;
      // on the top
      robot->place(osg::Matrix::rotate(-0.0*M_PI,0,1.0,0)*osg::Matrix::translate(0,0,initHeight+ 2*ii));

      // add delayed sensors
      std::list<SensorAttachment> sas = robot->getAttachedSensors();
      int k=0;
      for(auto& sa: sas){
        if(k%2==0){
          auto s = std::make_shared<DelaySensor>(sa.first,8);
          robot->addSensor(s);
          global.configs.push_back(s.get());
        }
        k++;
      }

      // one can also attach additional sensors
      // robot->addSensor(std::make_shared<SpeedSensor>(1.0, SpeedSensor::TranslationalRel, Sensor::X));
      // robot->addSensor(std::make_shared<SpeedSensor>(1.0, SpeedSensor::Translational, Sensor::Z));//World Coords


      AbstractController* sine = 0;
      if(useSine || useSwitch  ){
        // sine = new SineController(~0, SineController::Sine);
        sine = new MultiSineController(0x0FFF, SineController::Sine); // control the first 12
        sine->setParam("period", 25);
        sine->setParam("phaseshift", 0);
        if(noisecontrol>0)
          sine->setParam("amplitude",0);
        else{
          sine->setParam("amplitude",0.5);
        }
      }

      if(useDEP){
        DEPConf pc = DEP::getDefaultConf();
        pc.initFeedbackStrength=0.0;
        pc.calcEigenvalues = true;
        if(babbling) pc.initModel=false;

        DEP* dep = new DEP(pc);
        dep->setParam("epsM",0);
        dep->setParam("epsh",0.0);
        dep->setParam("synboost",2.2);
        dep->setParam("learningrule", 3);  // 3 is the DEPNEW rule

        // dep->setParam("indnorm",1); // 0 is global normalization
        // dep->setParam("urate",0.05);
        // dep->setParam("timedist",4);

        loadParams(dep, config_name);        

        controller=dep;
      } else if(useSine) {
        controller = sine;
      } else{
        assert("unknown controller");
      }
      if(useSwitch){
        std::list<AbstractController*> ctrls = {controller,sine};
        SwitchController* sw = new SwitchController(ctrls);
        controller = sw;
      }

      One2OneWiring* wiring = new One2OneWiring( whitenoise ? (NoiseGenerator*) new WhiteUniformNoise() : (NoiseGenerator*) new ColorUniformNoise(0.1));
      // NOTE: noise is by default switched off (noise=0 in global parameters)

      OdeAgent* agent = new OdeAgent(global);
      agent->init(controller, robot, wiring);
      // add an operator to keep robot from falling over
      agent->addOperator(new LimitOrientationOperator(Axis(0,0,1), Axis(0,0,1), M_PI*0.5, 30));
      // keep track of the robot's trajectory
      if(track) {
        TrackRobotConf trc = TrackRobot::getDefaultConf();
        if(trackfilename){
          trc.scene=trackfilename;
          trc.autoFilename = false;
        }
        agent->setTrackOptions(trc);
      }
      if(loadcontroller){
        if(agent->getController()->restoreFromFile(loadcontroller)){
          printf("loaded controller from %s\n", loadcontroller);
        }else{
          fprintf(stderr,"cannot load controller from %s\n", loadcontroller);
          simulation_time_reached=true;
        }
      }
      // learn model matrix
      if(babbling){
        MotorBabbler* babbler=new MotorBabbler();
        babbler->setParam("minperiod",20);
        babbler->setParam("maxperiod",200);
        babbler->setParam("amplitude",1);
        babbler->setParam("resample",200);
        global.configs.push_back(babbler);
        agent->startMotorBabblingMode(babbling,babbler);
        agent->fixateRobot(global,-1,babbling/50);
      }else{
        setModel(agent);
      }
      if(randomctrl) setRndController(agent);

      global.agents.push_back(agent);
      global.configs.push_back(agent);

      if(fix) robot->fixate(global);

      // if we use sine controller for manual control or a switching controller
      if(useSine || useSwitch){
        sine->setParam("phaseshift0", -1.2);
        sine->setParam("phaseshift1", 0);
        sine->setParam("phaseshift2", 0.8);
        sine->setParam("phaseshift3", 2);
        sine->setParam("phaseshift4", 0.8);
        sine->setParam("phaseshift5", 2);
        sine->setParam("phaseshift6", -1.2);
        sine->setParam("phaseshift7", 0);
        sine->setParam("phaseshift8", -1.2);
        sine->setParam("phaseshift9", 0);
        sine->setParam("phaseshift10",0.8);
        sine->setParam("phaseshift11",2);

        sine->setParam("amplitude0", 0.8);
        sine->setParam("amplitude2", 0.8);
        sine->setParam("amplitude4", 0.8);
        sine->setParam("amplitude6", 0.8);
        sine->setParam("amplitude8", 0.8);
        sine->setParam("amplitude10", 0.8);
      }
    }
    
  }

  virtual void setModel(OdeAgent* agent){
    /* Hexapod leg number
          \  /
           ||
      4--|~~~~|--5
         |    |
      2--|    |--3
         |    |
      0--|____|--1
      Each leg has up/down, front/back and tebia (knee)
      */
    // set connections into model
    DEP* dep = dynamic_cast<DEP*>(agent->getController());
    if(walkmodel && dep){
      Matrix M = dep->getM();
      // LC is transposed for historical reasons
      Matrix LC(M.getN(), M.getM());
      /// TRIPOD
      if(tripod){
        for(int k=0; k<2; k++){
          // leg 0: 3,4
          LC.val(3*3+k,0*3+k)=1;
          LC.val(4*3+k,0*3+k)=1;
          // leg 1: 2,5
          LC.val(2*3+k,1*3+k)=1;
          LC.val(5*3+k,1*3+k)=1;
          // leg 2: 1,5
          LC.val(1*3+k,2*3+k)=1;
          LC.val(5*3+k,2*3+k)=1;
          // leg 3: 0,4
          LC.val(0*3+k,3*3+k)=1;
          LC.val(4*3+k,3*3+k)=1;
          // leg 4: 0,3
          LC.val(0*3+k,4*3+k)=1;
          LC.val(3*3+k,4*3+k)=1;
          // leg 5: 1,2
          LC.val(1*3+k,5*3+k)=1;
          LC.val(2*3+k,5*3+k)=1;
        }
      }
      // subsequent legs on one side are negatively coupled (antiphasic)
      if(tripod_neg){
        for(int k=1; k<2; k++){ // front/back only
          // leg 0: - 2
          LC.val(2*3+k,0*3+k)=-1;
          // leg 1: - 3
          LC.val(3*3+k,1*3+k)=-1;
          // leg 2:  -4
          LC.val(4*3+k,2*3+k)=-1;
          // leg 3:  -5
          LC.val(5*3+k,3*3+k)=-1;
          // leg 4:  -5
          //LC.val(5*3+k,4*3+k)=-1;
          // leg 5:  -4
          //LC.val(4*3+k,5*3+k)=-1;
        }
      }
      // left and right leg pairs are negatively coupled (antiphasic)
      if(lateral_neg){
        for(int k=1; k<2; k++){// only front/back
          // leg 0: 1
          LC.val(1*3+k,0*3+k)=-1;
          // leg 1: 0
          LC.val(0*3+k,1*3+k)=-1;
          // leg 2: 3
          LC.val(3*3+k,2*3+k)=-1;
          // leg 3: 2
          LC.val(2*3+k,3*3+k)=-1;
          // leg 4: 5
          LC.val(5*3+k,4*3+k)=-1;
          // leg 5: 4
          LC.val(4*3+k,5*3+k)=-1;
        }
      }

      // delays (the delay sensors start with index 18 and for each leg we have 2,
      //  but we use only one for the connection
      if(walkdelay){
        for(int k=0; k<6; k++){
          LC.val(18+k*2+1,k*3)=1;
        }
      }

      if(legdelay){
        int k = 1;
        // leg 0: - 2
        LC.val(18+2*2+k,0*3+k)=1;
        // leg 1: - 3
        LC.val(18+3*2+k,1*3+k)=1;
        // leg 2:  -4
        LC.val(18+4*2+k,2*3+k)=1;
        // leg 3:  -5
        LC.val(18+5*2+k,3*3+k)=1;
      }

      std::cout << "apply Leg coupling" << std::endl;
      dep->setM(M+(LC^T));
    }
  }


  virtual void setRndController(OdeAgent* agent){
    DEP* dep = dynamic_cast<DEP*>(agent->getController());
    if(dep){
      Matrix C = dep->getC();
      dep->setC(C.map(random_minusone_to_one));
      printf("Set random controller matrix\n");
    }
  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    //set title containing inportant information
    DEP* dep = dynamic_cast<DEP*>(globalData.agents[0]->getController());
    double boost = dep->getParam("synboost");
    int Time = dep->getParam("Time");

    char config_chars[50] = {0};    
    sprintf(config_chars, "Synboost: %.1f, Time: %d" //, Bin:(%d,%d), Z: %.2f" 
      ,boost, Time);
    std::string config_string(config_chars);
    setTitle(/*"Cov: " + to_string(coverage) +*/ config_string);

  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Sim: m","reset model matrix");
  };

  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& global,
                       int key, bool down) {
    if(down){
      switch(key){
      case 'm' :{ // set model
        setModel(global.agents[0]);
        break;
      }
      case 'b' :{  //change the synchronized boost
        DEP* dep = dynamic_cast<DEP*>(global.agents[0]->getController());
        double boost = dep->getParam("synboost");
        dep->setParam("synboost", boost+0.1);
        std::cout<< "new boost is: " << dep->getParam("synboost")<< std::endl;
        break;
      }
      case 'B' :{  //change the synchronized boost
        DEP* dep = dynamic_cast<DEP*>(global.agents[0]->getController());
        double boost = dep->getParam("synboost");
        dep->setParam("synboost", boost-0.1);
        std::cout<< "new boost is: " << dep->getParam("synboost")<< std::endl;
        break;
      }
      case 't' :{  //change the Time
        DEP* dep = dynamic_cast<DEP*>(global.agents[0]->getController());
        int time = dep->getParam("Time");
        dep->setParam("Time", time+5);
        std::cout<< "new Time is: " << dep->getParam("Time")<< std::endl;
        break;
      }
      case 'T' :{  //change the Time
        DEP* dep = dynamic_cast<DEP*>(global.agents[0]->getController());
        int time = dep->getParam("Time");
        dep->setParam("Time", time-5);
        std::cout<< "new Time is: " << dep->getParam("Time")<< std::endl;
        break;
      }

      case 's' :{  //change the synchronized boost
        global.odeConfig.setParam("realtimefactor", 1);
        break;
      }


      }
    }
    return false;
  };

  virtual void usage() const {
    printf("  --------------- Specific settings for this simulation ------------\n");
    printf("    -walkmodel\tinitialize model to contain additional connections, see following switches:\n");
    printf("    -walkdelay\tadd connections for leg oscillation\n");
    printf("    -tripod\tadd connections for full tripod gait\n");
    printf("    -tripod_neg\tadd only negative connections for subsequent legs for tripod\n");
    printf("    -lateral_neg\tadd negative connections for lateral leg pairs\n");
    printf("    -leg_delay\tadd connections to delay subsequent legs on each side\n");
    printf("              END OF MODEL SWITCHES\n");
    printf("    -babbling STEPS\tstart with # STEPS in motor babbling mode (robot fixed)\n");
    printf("    -boxpile\tadd a boxpile to the scene\n");
    printf("    -randomctrl\tinitialize the C matrix randomly (with urate=0 this is static)\n");
    printf("  --------------- Misc stuff ------------\n");
    printf("    -numwalls N\tadd N surrounding walls around robot\n");
    printf("    -track FILENAME\trecord trajectory of robot into file\n");
    printf("    -loadcontroler FILE \tload the controller at startup\n");
    printf("    -height HEIGHT \tset initial height of robot\n");
    printf("    -noisecontrol TAU \tuse noise as motor commands with correlation length 1/TAU \n");
    printf("    -sine\tuse sine controller\n");
    printf("    -whitenoise\tuse white noise instead of colored noise for sensor noise (only active if paramter noise!=0)\n");
  };

};

int main (int argc, char **argv)
{
  int index = Simulation::contains(argv,argc,"-numwalls");
  if(index >0 && argc>index){
    numwalls=atoi(argv[index]);
  }
  track      = Simulation::contains(argv,argc,"-track")   != 0;
  index = Simulation::contains(argv,argc,"-trackfile");
  if(index >0 && argc>index){
    trackfilename=argv[index];
  }
  index = Simulation::contains(argv,argc,"-babbling");
  if(index >0 && argc>index){
    babbling=atoi(argv[index]);
  }
  index = Simulation::contains(argv,argc,"-loadcontroller");
  if(index >0 && argc>index){
    loadcontroller=argv[index];
  }
  index = Simulation::contains(argv,argc,"-height");
  if(index >0 && argc>index){
    initHeight=atof(argv[index]);
  }
  useSine    = Simulation::contains(argv,argc,"-sine")    != 0;
  fix        = Simulation::contains(argv,argc,"-fix")     != 0;
  useSwitch  = Simulation::contains(argv,argc,"-switch")!= 0;
  whitenoise = Simulation::contains(argv,argc,"-whitenoise")!= 0;
  walkmodel  = Simulation::contains(argv,argc,"-walkmodel")!= 0;
  walkdelay  = Simulation::contains(argv,argc,"-walkdelay")!= 0;
  tripod     = Simulation::contains(argv,argc,"-tripod")!= 0;
  tripod_neg = Simulation::contains(argv,argc,"-tripod_neg")!= 0;
  lateral_neg = Simulation::contains(argv,argc,"-lateral_neg")!= 0;
  legdelay   = Simulation::contains(argv,argc,"-legdelay")!= 0;
  boxpile    = Simulation::contains(argv,argc,"-boxpile")!= 0;
  randomctrl = Simulation::contains(argv,argc,"-randomctrl")!= 0;

  index = Simulation::contains(argv,argc,"-noisecontrol"); // 1/(correlation length)
  if(index >0 && argc>index){
    noisecontrol=atof(argv[index]);
    useSine=true; useDEP = false;
  }

  config_name = "config.txt";
  index = Simulation::contains(argv, argc, "-config");
  if(index)
    if(argc > index)
      config_name = argv[index];
  std::cout<< std::endl << config_name << " successfully parsed from the CML!"<< std::endl;

  // DEP is default controller
  if(useSine)
    useDEP = false;

  ThisSim sim;
  return sim.run(argc, argv) ? 0 :  1;
}
