echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j1

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j1

cd ../../../

if [ ! -f vocabulary/voc/ORBvoc.txt ]; then
echo "Uncompress vocabulary ..."
cd vocabulary/voc
tar -xf ORBvoc.txt.tar.gz
cd ../..
fi

echo "vocabulary uncompressed ..."

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j1
