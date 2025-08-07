#!/bin/bash 

cd build
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./../lib
ctest

lcov -d ../build -b . -c -o all.info
lcov -r all.info '/usr/*' '*/.conan/*' '*/build/*' '*/zos/*' \
    -o coverage.info
genhtml coverage.info -o UnitTestCoverageReport

rm -rf ../test/UnitTestCoverageReport
rm -rf ../test/UnitTestReport
mkdir ../test/UnitTestCoverageReport
mkdir ../test/UnitTestReport
cp -r UnitTestCoverageReport ../test/
cp -r bin/UnitTestReport ../test/
