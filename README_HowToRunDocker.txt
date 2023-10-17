Open DockerCLI
Open cmd prompt
cd to the location of Dockerfile (ex. cd C:\Users\willi\STM32CubeIDE\workspace_1.13.1\c_unit_test)
docker build -t unit-tests-image -f Dockerfile .
docker run --rm unit-tests-image (oppure RUN in DockerCLI)