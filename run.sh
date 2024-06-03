git clone https://gitlab.com/nsnam/ns-3-dev.git
cp -r files/src ns-3-dev
cd ns-3-dev
./ns3 configure --build-profile=optimized 
./ns3
