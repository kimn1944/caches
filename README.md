# Caches
The following project is an implementation of data and instruction caches for a 5 stage MIPS processor in Verilog.

## Implementation
* Instruction Cache: 32KB read only, direct mapped, 32B block size, 1 cycle latency and 10 cycle miss penalty cache.
* Data Cache: 32KB, write-back, write allocate, 2-way, 32B block size, 1 cycle latency and 10 cycle miss penalty cache.

## Getting Started
These instructions will tell you how to get the project up and running for testing.  

### Structure and prerequisites
In order to run the simulator you must download, extract and compile the files with the provided make file.
The extracted directory will have the following structure:

>verilog/ - 
this contains the verilog design files for the processor  
>sim_main/ - 
this contains the c++ source files necessary to run the simulator  
>tests/ - 
this contains the test files for the simulator    

### Running the simulator
In order to test the simulator you will have to call the executable and provide it with one of the test files like so:

>./VMIPS -f tests/cpp/hello -d X

where hello is the test file and X is the number of cycles you want the processor to run for. In our case 150,000.  
All the different test files are available in the test directory. You must run the file without any extensions. .txt and .cpp or .asm extended files were provided for debugging purposes only. In order for the test files to execute completely they must run for a sufficient amount of cycles. To ensure that, you must run every test file for up to 500,000 cycles.

### Results
The output of the test file will be printed to either stdout.txt or out.txt files. The amount of cycles the program took and the total instruction count will be provided in the termianl window. 

## Authors
**Nikita Kim**  
**Celine Wang**  
