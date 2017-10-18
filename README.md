# AIRSIM
1. Unreal account created with github req
2. clone Unreal

          #go to folder where you clone GitHub projects
          git clone -b 4.17 https://github.com/EpicGames/UnrealEngine.git
          cd UnrealEngine
          #few times Epic folks broke the build so we will get commit that works
          git checkout af96417313a908b20621a443175ba91683c238c8
          ./Setup.sh
          ./GenerateProjectFiles.sh
          make

**octo@octo:~/myAIRSIM/UnrealEngine$**

*. To open Unreal Editor
cd Engine/Binaries/Linux/ 
./UE4Editor
octo@octo:~/myAIRSIM/UnrealEngine/Engine/Binaries/Linux$ ./UE4Editor


3. clone AirSIM and build

          # go to folder where you clone GitHub projects
          git clone https://github.com/Microsoft/AirSim.git
          cd AirSim
          ./setup.sh
          ./build.sh
          
**octo@octo:~/myAIRSIM$**
**octo@octo:~/myAIRSIM/AirSim$**



The simulation framework includes an API for downloading data collected from GPS and other sensors, including visual data, and to control vehicles. The project includes Windows builds, and Microsoft will soon make Linux ones available. Other platforms are easily supported because the code is built with cross-platform in mind, making use of CMake. Microsoft works on supporting ZeroMQ and Protobuf, and other languages such as Python.


#### REF

https://github.com/Microsoft/AirSim

https://github.com/Microsoft/AirSim/blob/master/docs/using_car.md

https://github.com/Microsoft/AirSim/blob/master/docs/build_linux.md

https://www.microsoft.com/en-us/research/project/aerial-informatics-robotics-platform/

https://dev.px4.io/en/simulation/airsim.html

https://docs.unrealengine.com/latest/INT/Platforms/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/4/index.html
