sudo apt install cmake -y 
git clone https://gitlab.com/nsnam/ns-3-dev.git
cp -r src ns-3-dev
cp SimulatorScript.cc ns-3-dev/scratch
cp ns3testbed.cc.cc ns-3-dev/scratch

cd ns-3-dev
./ns3 configure --build-profile=optimized 
./ns3
