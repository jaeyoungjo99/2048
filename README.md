# 2048
### 2048 game with c++ and OpenGL

![2048](docs/2048_example.png)


# Install dependencies
```bash
$ sudo apt-get install libglfw3-dev
$ sudo apt-get install libglew-dev
$ sudo apt-get install freeglut3-dev
$ sudo apt-get install libtbb-dev
$ sudo apt-get install libeigen3-dev
```

# Clone the package 
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/jaeyoungjo99/2048.git
```

# Build the package
```bash
mkdir build
cd build
cmake .. -G "MinGW Makefiles"
mingw32-make

```

# 정적 빌드 확인
ldd game_2048.exe

# Run the package
```bash
$ source devel/setup.bash
$ roslaunch game_2048 game_2048.launch
