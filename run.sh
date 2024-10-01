sudo apt install cmake -y 
git clone https://gitlab.com/nsnam/ns-3-dev.git
cp -r src ns-3-dev
cd ns-3-dev
./ns3 configure --build-profile=optimized 
./ns3
