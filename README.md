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
3. clone AirSIM and build

          # go to folder where you clone GitHub projects
          git clone -b 4.17 https://github.com/EpicGames/UnrealEngine.git
          cd UnrealEngine
          # few times Epic folks broke the build so we will get commit that works
          git checkout af96417313a908b20621a443175ba91683c238c8
          ./Setup.sh
          ./GenerateProjectFiles.sh
          make








#### REF

https://github.com/Microsoft/AirSim
https://github.com/Microsoft/AirSim/blob/master/docs/using_car.md
https://github.com/Microsoft/AirSim/blob/master/docs/build_linux.md
