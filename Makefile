include $(SOC_LIB_DIR)/hardware.config

SCHED_LIB_DIR = $(SOC_LIB_DIR)/sched_library
TASK_LIB_DIR = $(SOC_LIB_DIR)/task_library

CPU ?= ariane
ARCH ?= riscv
ifdef DO_CROSS_COMPILATION
 CROSS_COMPILE ?= riscv64-unknown-linux-gnu-
 TOOLCHAIN_PREFIX ?= $(RISCV_BIN_DIR)/riscv64-unknown-linux-gnu-
 EXE_EXTENSION=-RV
 CROSS_COMPILE_FLAGS = --target=riscv64 -march=rv64g -mabi=lp64d
endif

ifdef COMPILE_TO_ESP
 ESP_DRIVERS ?= $(ESP_ROOT)/soft/common/drivers
 ESP_DRV_LINUX  = $(ESP_DRIVERS)/linux
endif

# CC = gcc -std=c++11
CC = gcc -std=c99

HPVM_BIN_PATH = $(HPVM_BUILD_DIR)/bin
OPT = $(HPVM_BIN_PATH)/opt
LLVM_LINK = $(HPVM_BIN_PATH)/llvm-link
HPVMCC = $(HPVM_BIN_PATH)/clang
HPVMCXX = $(HPVM_BIN_PATH)/clang++
HPVM_RT_PATH = $(HPVM_BUILD_DIR)/tools/hpvm/projects/hpvm-rt
HPVM_RT_LIB = $(HPVM_RT_PATH)/hpvm-rt.bc
HCC=$(HPVM_BIN_PATH)/hpvm-extract-task
HPVM_DEF_FILE=./soc_utils/HPVMCFunctionDeclarations.ll

YEL='\033[0;33m'
NC='\033[0m'



#INCDIR ?=
INCDIR += -I./include -I./soc_utils -I$(SCHED_LIB_DIR)/include -I$(TASK_LIB_DIR)/include
ifdef COMPILE_TO_ESP
 INCDIR += -I$(ESP_DRIVERS)/common/include
 INCDIR += -I$(ESP_DRIVERS)/linux/include
endif

CFLAGS ?= -O2 #-g
CFLAGS += $(INCDIR)
CFLAGS += -DINT_TIME
ifdef COMPILE_TO_ESP
 CFLAGS += -DCOMPILE_TO_ESP
endif
#  -- ALWAYS use this one! --   ifdef CONFIG_ESP_INTERFACE
CFLAGS += -DUSE_ESP_INTERFACE
#   endif

# This sets the maximum number of any Acceleartor type in this build
#  This should be equal to the largest number of any of FFT, VIT, CV, etc.
#  But it could be a larger value as well
#ifdef CONFIG_MAX_ACCEL_ANY_TYPE
# CFLAGS += -DMAX_ACCEL_OF_EACH_TYPE=$(CONFIG_MAX_ACCEL_ANY_TYPE)
#endif

# This selects the number of each Acceleartor type:
# MAX for the CPU is up to you -- that is "spawned CPU threads"
# MAX for others is determined by the hardware run on
ifdef CONFIG_NUM_CPU
 CFLAGS += -DNUM_CPU_ACCEL=$(CONFIG_NUM_CPU)
endif
ifdef CONFIG_NUM_VIT
 CFLAGS += -DNUM_VIT_ACCEL=$(CONFIG_NUM_VIT)
endif
ifdef CONFIG_NUM_FFT
 CFLAGS += -DNUM_FFT_ACCEL=$(CONFIG_NUM_FFT)
endif
ifdef CONFIG_NUM_CV
 CFLAGS += -DNUM_CV_ACCEL=$(CONFIG_NUM_CV)
endif

# This adds the code to allow the user to specify (at run-time)
#  some additional base-level (not-critical) tasks of Vit, CV, FFT
ifdef ALLOW_ADD_BASE_TASKS
 CFLAGS += -DHPVM_BASE_CRIT
endif

CFLAGS += -DMAX_RADAR_LOGN=$(LOG2_MAX_FFT_SAMPLES)

SW_STR = S
FA_STR =
VA_STR =
CA_STR =
CB_STR =
ifdef CONFIG_FFT_EN
 SW_STR =
 FA_STR = F$(CONFIG_FFT_ACCEL_VER)
 CFLAGS += -DHW_FFT
 CFLAGS += -DUSE_FFT_FX=$(CONFIG_FFT_FX)
 CFLAGS += -DUSE_FFT_ACCEL_VERSION=$(CONFIG_FFT_ACCEL_VER)
 CFLAGS += -DFFT_DEV_BASE='"$(FFT_DEVICE_BASE)"'
endif
ifdef CONFIG_FFT_BITREV
 CFLAGS += -DHW_FFT_BITREV
endif

ifdef CONFIG_VITERBI_EN
 SW_STR =
 VA_STR = V
 CFLAGS += -DHW_VIT
 CFLAGS += -DVIT_DEV_BASE='"$(VIT_DEVICE_BASE)"'
endif

ifdef CONFIG_KERAS_CV_BYPASS
 CFLAGS += -DBYPASS_KERAS_CV_CODE
else
 CFLAGS += -I/usr/include/python3.6m
endif

ifdef CONFIG_CV_EN
 SW_STR =
 CA_STR = CH
 CFLAGS += -DHW_CV -DENABLE_NVDLA -I/usr/include/python3.6m
#  CFLAGS += -DCNN_DEV_BASE='"$(CNN_DEVICE_BASE)"' -I/usr/include/python3.6m
else # Not using the HWR accelerator
 ifdef CONFIG_FAKE_CV_EN
  SW_STR =
  CA_STR = CF
  CFLAGS += -DFAKE_HW_CV
 endif
endif

ifdef CONFIG_CV_ONLY_HWR
 SW_STR =
 CB_STR = o
 CFLAGS += -DHW_ONLY_CV
endif

ifdef SL_VIZ
CFLAGS += -DSL_VIZ
endif

ifdef CONFIG_VERBOSE
CFLAGS += -DVERBOSE
OPTFLAGS += -debug
endif

ifdef HPVM_BASE_CRIT
CFLAGS += -DHPVM_BASE_CRIT
endif

ifdef CONFIG_DBG_THREADS
CFLAGS += -DDBG_THREADS
endif

ifdef CONFIG_SUPER_VERBOSE
CFLAGS += -DSUPER_VERBOSE
endif

ifdef CONFIG_FUSED_MAP
CFLAGS += -DWRITE_FUSED_MAPS
endif
ifdef CONFIG_GDB
CFLAGS += -g
endif

ifdef COMPILE_TO_ESP
CFLAGS += -DHPVM -DDEVICE=EPOCHS_TARGET -fno-exceptions
endif


ifdef COMPILE_HCC
CFLAGS += -DHCC
endif



SCHED_LIB = $(SCHED_LIB_DIR)/libscheduler.a
TASK_LIB = $(TASK_LIB_DIR)/libtasks.a

SCHED_MODULE = $(SCHED_LIB_DIR)/libscheduler.bc
TASK_MODULE = $(TASK_LIB_DIR)/libtasks.bc

LDLIBS ?=
ifdef COMPILE_TO_ESP
 ESP_BUILD_DRIVERS=$(SCHED_LIB_DIR)/esp-build/drivers
 LDLIBS += -L$(ESP_BUILD_DRIVERS)/contig_alloc
 LDLIBS += -L$(ESP_BUILD_DRIVERS)/test
 LDLIBS += -L$(ESP_BUILD_DRIVERS)/libesp
endif
#ifndef CONFIG_KERAS_CV_BYPASS
# LDLIBS += 
#endif
LDLIBS += -L$(TASK_LIB_DIR) -L$(SCHED_LIB_DIR)

LDFLAGS ?=
LDFLAGS += -ltasks -lscheduler
LDFLAGS += -lm
LDFLAGS += -lpthread
LDFLAGS += -lboost_graph -lboost_regex
LDFLAGS += -ldl -rdynamic
ifdef COMPILE_TO_ESP
 LDFLAGS += -lrt
 LDFLAGS += -lesp
 LDFLAGS += -ltest
 LDFLAGS += -lcontig
endif
ifndef CONFIG_KERAS_CV_BYPASS
 LDFLAGS += -lpython3.6m
endif

SRC_T := $(foreach f, $(wildcard src/*.c), $(shell basename $(f)))
SRC_D = $(wildcard src/*.c)
HDR_T = $(wildcard include/*.h)
OBJ_T = $(SRC_T:%.c=obj_t/%.o)
OBJ_S = $(SRC_T:%.c=obj_s/%.o)
OBJ_HPVM = obj_hpvm/hpvm_src_main.ll obj_hpvm/hpvm_src_xmit_pipe.ll obj_hpvm/hpvm_src_occgrid.ll obj_hpvm/hpvm_src_recv_pipe.ll
#OBJ_LL = $(SRC_T:%.c=obj_hpvm/%.ll)
OBJ_LL = $(filter-out $(OBJ_HPVM), $(SRC_T:%.c=obj_hpvm/%.ll))
OBJ_H = obj_hpvm/hpvm_tasks.host.ll 
OBJ_LINKED = obj_hpvm/hpvm_tasks.linked.ll

VPATH = ./src

TARGET=test-scheduler$(EXE_EXTENSION)-$(SW_STR)$(FA_STR)$(VA_STR)$(CA_STR)$(CB_STR)-P$(CONFIG_NUM_CPU)V$(CONFIG_NUM_VIT)F$(CONFIG_NUM_FFT)N$(CONFIG_NUM_CV)
STARGET=sim-test-scheduler$(EXE_EXTENSION)-$(SW_STR)$(FA_STR)$(VA_STR)$(CA_STR)$(CB_STR)-P$(CONFIG_NUM_CPU)V$(CONFIG_NUM_VIT)F$(CONFIG_NUM_FFT)N$(CONFIG_NUM_CV)
HPVMTARGET=hpvm-test-scheduler$(EXE_EXTENSION)-$(SW_STR)$(FA_STR)$(VA_STR)$(CA_STR)$(CB_STR)-P$(CONFIG_NUM_CPU)V$(CONFIG_NUM_VIT)F$(CONFIG_NUM_FFT)N$(CONFIG_NUM_CV)
HPVMTARGET_CPU=hpvm-test-cpu$(EXE_EXTENSION)-$(SW_STR)$(FA_STR)$(VA_STR)$(CA_STR)$(CB_STR)-P$(CONFIG_NUM_CPU)V$(CONFIG_NUM_VIT)F$(CONFIG_NUM_FFT)N$(CONFIG_NUM_CV)


$(info $$EXE_EXTENSION is [${EXE_EXTENSION}])
$(info $$TARGET is [${TARGET}])
$(info $$STARGET is [${STARGET}])

$(info $$OBJ_LL is [${OBJ_LL}])
$(info $$OBJ_H is [${OBJ_H}])
$(info $$OBJ_LINKED is [${OBJ_LINKED}])

ifdef CONFIG_CV_EN
NVDLA_MODULE=hpvm-mod.nvdla
endif 
$(info $$NVDLA_MODULE is [${NVDLA_MODULE}])

#all: $(TARGET) $(STARGET)
all: $(TARGET)

#hpvm-epochs: CFLAGS += -DHPVM -DDEVICE=EPOCHS_TARGET
#hpvm-epochs: CFLAGS := $(filter-out -I$(SCHED_LIB_DIR)/include -I$(TASK_LIB_DIR)/include, $(CFLAGS))
#hpvm-epochs: CFLAGS := $(filter-out -I$(SCHED_LIB_DIR)/include, $(CFLAGS))
hpvm-epochs: CONFIG_FILE ?= soc_utils/config_files/base_me_p2.config 
hpvm-epochs: TASK_CONFIG_FILE ?= $(TASK_LIB_DIR)/task_lib.config 
hpvm-epochs: BACKEND_LOAD = -load LLVMDFG2LLVM_EPOCHS.so
hpvm-epochs: BACKEND_FLAG = -dfg2llvm-epochs -sched-lib-path=$(SCHED_MODULE) -sched-config=$(CONFIG_FILE) -task-config=$(TASK_CONFIG_FILE) -task-lib-path=$(TASK_MODULE)
hpvm-epochs: HPVM = hpvm

hpvm-cpu: CFLAGS += -DHPVM -DDEVICE=CPU_TARGET
#hpvm-cpu: CFLAGS := $(filter-out -I$(SCHED_LIB_DIR)/include -I$(TASK_LIB_DIR)/include, $(CFLAGS))
hpvm-cpu: CFLAGS := $(filter-out -I$(SCHED_LIB_DIR)/include, $(CFLAGS))
hpvm-cpu: BACKEND_LOAD = -load LLVMDFG2LLVM_CPU.so
hpvm-cpu: BACKEND_FLAG = -dfg2llvm-cpu 
hpvm-cpu: LDFLAGS := $(filter-out -ltasks -lscheduler, $(LDFLAGS))
hpvm-cpu: LDLIBS := $(filter-out -L$(TASK_LIB_DIR) -L$(SCHED_LIB_DIR), $(LDLIBS))


hpvm-epochs: check_env obj_hpvm $(SCHED_MODULE) $(TASK_MODULE) $(NVDLA_MODULE) $(HPVMTARGET)
hpvm-cpu: check_env obj_hpvm $(HPVMTARGET_CPU)


#------------------------------------------------------------------------------------------

#------------------------------------------------------------------------------------------


CUR_DIR = $(dir $(realpath $(firstword $(MAKEFILE_LIST))))
$(info $$CUR_DIR is [${CUR_DIR}])

ROOT := $(CUR_DIR)/sw/umd
TOP := $(ROOT)
$(info $$ROOT is [${ROOT}])

ESP_NVDLA_DIR = esp_hardware/nvdla
#INCLUDES +=  -I$(SRC_DIR) -I$(ESP_NVDLA_DIR) -I$(TOP)/core/include
INC_DIR +=  -I$(ESP_NVDLA_DIR) -I$(ROOT)/core/include

NVDLA_MAKE_DIR = $(HPVM_DIR)/test/epoch_dnn
NVDLA_RES_DIR = $(NVDLA_MAKE_DIR)/gen_miniera

NVDLA_RUNTIME_DIR = $(ROOT)
NVDLA_RUNTIME = $(NVDLA_RUNTIME_DIR)/out
$(info $$NVDLA_RUNTIME is [${NVDLA_RUNTIME}])
$(info $$NVDLA_RUNTIME_DIR is [${NVDLA_RUNTIME_DIR}])

ifdef CONFIG_CV_EN

MODULE := nvdla_runtime

include $(ROOT)/make/macros.mk

BUILDOUT ?= $(ROOT)/out/apps/runtime
BUILDDIR := $(BUILDOUT)/$(MODULE)
TEST_BIN := $(BUILDDIR)/$(MODULE)

MODULE_COMPILEFLAGS := -W -Wall -Wno-multichar -Wno-unused-parameter -Wno-unused-function -Werror-implicit-function-declaration
MODULE_CFLAGS := --std=c99
MODULE_CPPFLAGS := --std=c++11 -fexceptions -fno-rtti
NVDLA_FLAGS := -pthread -L$(ROOT)/external/ -ljpeg -L$(ROOT)/out/core/src/runtime/libnvdla_runtime -lnvdla_runtime  -Wl,-rpath=.

include esp_hardware/nvdla/rules.mk
endif

$(info $$ALLMODULE_OBJS is [${ALLMODULE_OBJS}])
$(info $$MODULE_OBJS is [${MODULE_OBJS}])
$(info $$CFLAGS is [${CFLAGS}])
#-----------------------------------------------------------------------------------------------

$(NVDLA_MODULE):
	@echo -e ${YEL}Compiling NVDLA Runtime Library${NC}
	@cd $(NVDLA_RUNTIME_DIR) && make ROOT=$(ROOT) runtime
	@echo -e ${YEL}Compiling HPVM Module for NVDLA${NC}
	cd $(NVDLA_MAKE_DIR) && ln -sf $(CUR_DIR)/user_scripts/miniera.py . && python miniera.py && cp $(NVDLA_RES_DIR)/$(NVDLA_MODULE) $(CUR_DIR)
	#@cd $(NVDLA_MAKE_DIR) && python gen_me_hpvm_mod.py && cp $(NVDLA_RES_DIR)/$(NVDLA_MODULE) $(CUR_DIR)

#-----------------------------------------------------------------------------------------------
obj_t/%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -c $< -o $@ $(LDFLAGS)

obj_s/%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -DUSE_SIM_ENVIRON -c $< -o $@


$(OBJ_T): $(HDR_T)

$(TARGET): $(TASK_LIB) $(SCHED_LIB) obj_t $(OBJ_T) $(NVDLA_MODULE) $(ALLMODULE_OBJS)
	#$(CROSS_COMPILE)$(CC) -fPIC $< -c -o me_test.o
	$(CROSS_COMPILE)$(LD) -r $(ALLMODULE_OBJS) $(OBJ_T) -o wnvdla_test.o
	$(CROSS_COMPILE)$(CXX) $(LDLIBS) wnvdla_test.o -o $@ $(LDFLAGS) $(NVDLA_FLAGS)
	#$(CROSS_COMPILE)$(CC) $(LDLIBS) $(OBJ_T) -o $@ $(LDFLAGS)

$(STARGET): $(TASK_LIB) $(SCHED_LIB) obj_s $(OBJ_S)
	$(CROSS_COMPILE)$(CC) $(LDLIBS) $(OBJ_S) -o $@ $(LDFLAGS)

$(HPVMTARGET): $(OBJ_LINKED) $(TASK_LIB) $(SCHED_LIB) $(ALLMODULE_OBJS)
	$(HPVMCC) $(CROSS_COMPILE_FLAGS) -fPIC $< -c -o me_test.o
	$(CROSS_COMPILE)$(LD) -r $(ALLMODULE_OBJS) me_test.o -o wnvdla_test.o
	$(CROSS_COMPILE)$(CXX) $(LDLIBS) wnvdla_test.o -o $@ $(LDFLAGS) $(NVDLA_FLAGS)
	rm me_test.o wnvdla_test.o

$(HPVMTARGET_CPU): $(OBJ_LINKED) $(TASK_LIB) $(SCHED_LIB) 
	$(HPVMCC) $(CROSS_COMPILE_FLAGS) -fPIC $< -c -o me_test.o
	$(CROSS_COMPILE)$(CXX) $(LDLIBS) me_test.o -o $@ $(LDFLAGS) $(CROSS_COMPILE_FLAGS)



obj_t:
	mkdir $@

obj_s:
	mkdir $@

obj_hpvm:
	mkdir $@

obj_hpvm/hpvm_src_%.ll : src/%.c
	$(HPVMCC) $(CFLAGS) -emit-llvm -S -o $@ $< $(OPTFLAGS)

# obj_hpvm/hpvm_xmit_pipe.ll : src/xmit_pipe.c
# 	$(HPVMCC) $(CFLAGS) -emit-llvm -S -o $@ $< $(OPTFLAGS)

obj_hpvm/hpvm_tasks.ll : $(OBJ_HPVM)
	$(LLVM_LINK) $^ -S -o $@

obj_hpvm/hpvm_tasks.hetero.ll : obj_hpvm/hpvm_tasks.ll
	$(HCC) -declsfile $(HPVM_DEF_FILE) --no-return-sizes -use0D $< -o $@ $(OPTFLAGS)

obj_hpvm/hpvm_tasks.hpvm.ll : obj_hpvm/hpvm_tasks.hetero.ll
	$(OPT) -load LLVMGenHPVM.so -genhpvm -globaldce -hpvm-timers-gen $< -S -o $@ $(OPTFLAGS)

$(OBJ_H) : obj_hpvm/hpvm_tasks.hpvm.ll
	$(OPT) -load LLVMBuildDFG.so $(BACKEND_LOAD) -load LLVMClearDFG.so $(BACKEND_FLAG) -clearDFG -dce -globaldce -S $< -o $@ $(OPTFLAGS)

# $(OBJ_LL):obj_hpvm/%.ll : src/%.c
# 	$(HPVMCC) $(CFLAGS) -emit-llvm -S -o $@ $<

$(OBJ_LINKED) : $(OBJ_H) $(HPVM_RT_LIB)
	$(LLVM_LINK) $^ -S -o $@


$(SCHED_LIB) $(SCHED_MODULE):
	@echo "***********************************"
	@echo "*** Compiling Scheduler Library ***"
	@echo "***********************************"
	(cd $(SCHED_LIB_DIR); $(MAKE) $(HPVM))
	@echo "***********************************"
	@echo "************** DONE! **************"
	@echo "***********************************"

$(TASK_LIB) $(TASK_MODULE):
	@echo "***********************************"
	@echo "****** Compiling Task Library *****"
	@echo "***********************************"
	(cd $(TASK_LIB_DIR); $(MAKE) $(HPVM))
	@echo "***********************************"
	@echo "************** DONE! **************"
	@echo "***********************************"

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

.PHONY: all clean clobber hpvm-epochs hpvm-cpu check_env 


#depend:;	makedepend -fMakefile -- $(CFLAGS) -- $(SRC_D)
# DO NOT DELETE THIS LINE -- make depend depends on it.

src/getopt.o: ./include/getopt.h
src/scheduler.o: ./include/getopt.h ./include/utils.h
src/scheduler.o: ./include/verbose.h
src/scheduler.o: ./include/scheduler.h ./include/base_types.h
src/scheduler.o: ./include/calc_fmcw_dist.h
src/main.o: ./include/getopt.h ./include/verbose.h ./include/scheduler.h
src/main.o: ./include/base_types.h
src/main.o: ./include/kernels_api.h ./include/calc_fmcw_dist.h
src/main.o: ./include/utils.h ./include/sim_environs.h
src/hpvm_tasks.o: ./include/hpvm_tasks.h ./include/base_types.h