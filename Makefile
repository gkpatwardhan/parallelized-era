# CC = gcc -std=c++11
CC = gcc -std=c99

HPVM_BIN_PATH = $(HPVM_BUILD_DIR)/bin
OPT = $(HPVM_BIN_PATH)/opt
LLVM_LINK = $(HPVM_BIN_PATH)/llvm-link
HPVMCC = $(HPVM_BIN_PATH)/clang
HPVMCXX = $(HPVM_BIN_PATH)/clang++
HPVM_RT_PATH = $(HPVM_BUILD_DIR)/tools/hpvm/projects/hpvm-rt
HPVM_RT_LIB = $(HPVM_RT_PATH)/hpvm-rt.bc
HCC = $(HPVM_BIN_PATH)/hcc
HPVM_DEF_FILE=./soc_utils/HPVMCFunctionDeclarations.ll

YEL='\033[0;33m'
NC='\033[0m'



#INCDIR ?=
INCDIR += -I./include -I./soc_utils -I$(SCHED_LIB_DIR)/include -I$(TASK_LIB_DIR)/include -I/usr/include/python3.6m -I./include/stb_image
CFLAGS ?= -O1 #-g
CFLAGS += $(INCDIR)
CFLAGS += -DINT_TIME


# These are the settings for the size of the Costmap, etc.
CONFIG_GRID_MAP_X_DIM=100
CONFIG_GRID_MAP_Y_DIM=100
CONFIG_GRID_MAP_RESLTN=2.0
CONFIG_RAYTR_RANGE=100

CFLAGS += -DGRID_MAP_X_DIM=$(CONFIG_GRID_MAP_X_DIM)
CFLAGS += -DGRID_MAP_Y_DIM=$(CONFIG_GRID_MAP_Y_DIM)
CFLAGS += -DGRID_MAP_RESLTN=$(CONFIG_GRID_MAP_RESLTN)
CFLAGS += -DRAYTR_RANGE=$(CONFIG_RAYTR_RANGE)


SRC_T := $(foreach f, $(wildcard src/*.c), $(shell basename $(f)))
SRC_D = $(wildcard src/*.c)
HDR_T = $(wildcard include/*.h)
OBJ_T = $(SRC_T:%.c=obj_t/%.o)
OBJ_S = $(SRC_T:%.c=obj_s/%.o)
OBJ_HPVM = $(SRC_T:%.c=obj_hpvm/hpvm_src_%.ll)
OBJ_LL = $(filter-out $(OBJ_HPVM), $(SRC_T:%.c=obj_hpvm/%.ll))
OBJ_HOST = obj_hpvm/hpvm_tasks.host.ll
OBJ_LINKED = obj_hpvm/hpvm_tasks.linked.ll

VPATH = ./src

HPVMTARGET_CPU=hpvm-test-cpu$(EXE_EXTENSION)-$(SW_STR)$(FA_STR)$(VA_STR)$(CA_STR)$(CB_STR)-P$(CONFIG_NUM_CPU)V$(CONFIG_NUM_VIT)F$(CONFIG_NUM_FFT)N$(CONFIG_NUM_CV)
HPVMTARGET_GPU=hpvm-test-gpu$(EXE_EXTENSION)-$(SW_STR)$(FA_STR)$(VA_STR)$(CA_STR)$(CB_STR)-P$(CONFIG_NUM_CPU)V$(CONFIG_NUM_VIT)F$(CONFIG_NUM_FFT)N$(CONFIG_NUM_CV)

all: $(TARGET)

hpvm-epochs: CONFIG_FILE ?= soc_utils/config_files/base_me_p2.config
hpvm-epochs: TASK_CONFIG_FILE ?= $(TASK_LIB_DIR)/task_lib.config
hpvm-epochs: BACKEND_LOAD = -load LLVMDFG2LLVM_EPOCHS.so
hpvm-epochs: BACKEND_FLAG = -dfg2llvm-epochs -sched-lib-path=$(SCHED_MODULE) -sched-config=$(CONFIG_FILE) -task-config=$(TASK_CONFIG_FILE) -task-lib-path=$(TASK_MODULE)
hpvm-epochs: HPVM = hpvm

hpvm-cpu: CFLAGS += -DHPVM -DDEVICE=CPU_TARGET
hpvm-cpu: BACKEND_LOAD = -load LLVMDFG2LLVM_CPU.so
hpvm-cpu: BACKEND_FLAG = -dfg2llvm-cpu

hpvm-gpu: CFLAGS += -DHPVM -DDEVICE=GPU_TARGET -DGPU
hpvm-gpu: BACKEND_LOAD = -load LLVMDFG2LLVM_CPU.so
hpvm-gpu: BACKEND_FLAG = -load LLVMLocalMem.so -load LLVMDFG2LLVM_OpenCL.so -load LLVMDFG2LLVM_CPU.so -load LLVMDFG2LLVM_OpenCL.so -localmem -dfg2llvm-cpu

OPT_FLAGS = -enable-new-pm=0  --debug -inline -load HPVMGenHPVM.so -genhpvm -globaldce -hpvm-timers-gen -load HPVMBuildDFG.so 

hpvm-cpu: OPT_FLAGS += -load HPVMDFG2LLVM_CPU.so -load HPVMClearDFG.so -load HPVMDFGTransformPasses.so  -sequentializeflatten -dfg2llvm-cpu -clearDFG -hpvm-timers-cpu 
hpvm-gpu: OPT_FLAGS += -load HPVMLocalMem.so -load HPVMDFG2LLVM_GPU_OCL.so -load HPVMDFG2LLVM_CPU.so -load HPVMClearDFG.so  -localmem -dfg2llvm-gpu-ocl -dfg2llvm-cpu -clearDFG -hpvm-timers-cpu -hpvm-timers-ptx 

hpvm-cpu: check_env obj_hpvm $(HPVMTARGET_CPU)
hpvm-gpu: check_env obj_hpvm $(HPVMTARGET_GPU)


$(info $$EXE_EXTENSION is [${EXE_EXTENSION}])
$(info $$TARGET is [${TARGET}])
$(info $$STARGET is [${STARGET}])

$(info $$OBJ_LL is [${OBJ_LL}])
$(info $$OBJ_HOST is [${OBJ_HOST}])
$(info $$OBJ_LINKED is [${OBJ_LINKED}])


# 0. Create the output directory for make
obj_hpvm:
	mkdir $@


# 1. Create a .ll file for all .c files in src/
obj_hpvm/hpvm_src_%.ll : src/%.c
	echo $<
	$(HPVMCC) $(CFLAGS) -fno-exceptions -emit-llvm -S -o $@ $<

# 2. Create a single .ll file from files generated in step 1
obj_hpvm/hpvm_tasks.ll : $(OBJ_HPVM)
	$(LLVM_LINK) $^ -S -o $@

# 3. Parse the hetero c++ flags in file generated in step 2
obj_hpvm/hpvm_tasks.hetero.bc : obj_hpvm/hpvm_tasks.ll
	$(HCC) -declsfile $(HPVM_DEF_FILE) -dot-dfg --no-return-sizes -use0D $< -o $@ 

# 4. Create .host file
$(OBJ_HOST) : obj_hpvm/hpvm_tasks.hetero.bc 
	$(OPT) $(OPT_FLAGS) -S $< -o $@




# 5. Link the .host file created in step 4 with hpvm-rt-lib
$(OBJ_LINKED) : $(OBJ_HOST) $(HPVM_RT_LIB)
	$(LLVM_LINK) $^ -S -o $@

# 6. Create executable file
$(HPVMTARGET_GPU): $(OBJ_LINKED)
	$(HPVMCXX) -O1 -lm -lm -lpthread -lrt -lOpenCL -L/software/cuda-9.1/lib64 $< -o $@ -L/usr/lib/python3.6/config-3.6m-x86_64-linux-gnu -L/usr/lib -lpython3.6m -lpthread -ldl  -lutil -lm  -Xlinker -export-dynamic -Wl,-O1 -Wl,-Bsymbolic-functions

$(HPVMTARGET_CPU): $(OBJ_LINKED)
	$(HPVMCXX) -O1 -lm -lm -lpthread -lrt -lOpenCL -L/software/cuda-9.1/lib64 $< -o $@ -L/usr/lib/python3.6/config-3.6m-x86_64-linux-gnu -L/usr/lib -lpython3.6m -lpthread -ldl  -lutil -lm  -Xlinker -export-dynamic -Wl,-O1 -Wl,-Bsymbolic-functions

clean:
	$(RM) $(OBJ_T) $(OBJ_S) $(OBJ_LL) $(OBJ_H) $(OBJ_LINKED)
	$(RM) -r obj_t obj_s obj_hpvm
	$(RM) *.dot
	$(RM) wnvdla_test.o
	if [ -f "$(NVDLA_MODULE)" ]; then rm $(NVDLA_MODULE); fi
	if [ -d "$(NVDLA_RUNTIME)" ]; then rm -rf $(NVDLA_RUNTIME); fi


clobber: clean
	$(RM) $(TARGET)
	$(RM) $(STARGET)
	$(RM) $(HPVMTARGET)
	$(RM) $(HPVMTARGET_CPU)


check_env:
ifndef HPVM_DIR
	$(error HPVM_DIR is undefined! setup_paths.sh needs to be sourced before running make!)
endif

.PHONY: all clean clobber hpvm-epochs hpvm-cpu hpvm-gpu check_env
