Debug = 0

#CXX=riscv64-eyenix-linux-gnu-g++

#BOOST_LDFLAG = -lboost_filesystem -lboost_thread -lboost_iostreams -lboost_chrono -lboost_system
#BOOST_LDFLAG = -L/data/guest/guest3/boost_install/lib -lboost_thread -lboost_iostreams -lboost_chrono -lboost_system
#BOOST_LDFLAG = -L$(SDKTARGETSYSROOT)/lib -lboost_thread -lboost_iostreams -lboost_chrono -lboost_system
BOOST_LDFLAG = -lboost_thread -lboost_iostreams -lboost_chrono -lboost_system -lboost_log -lboost_log_setup -lboost_coroutine

#LDFLAGS = -lpthread -lvsomeip3 $(BOOST_LDFLAG) -lrt
LDFLAGS = -lpthread $(BOOST_LDFLAG) -lrt
#CFLAGS = -Wall -O2 -std=c++14 -I/data/guest/guest3/boost_install/include
#CFLAGS = -Wall -O2 -std=c++14 -I$(SDKTARGETSYSROOT)/include
CFLAGS = -Wall -O0 -g -std=c++14 -DBOOST_LOG_DYN_LINK=1

#CFLAGS = -Wall -g -Os -DDEBUG
#INC_DIR = $(shell pkg-config --cflags sdl2)

SOURCES=main.cpp camera_response.cpp
OBJECTS=$(SOURCES:.cpp=.o)

SERVER = shm_serv
CLIENT = shm_cli
TEST_0 = test_0
TEST_1 = test_1
TEST_2 = test_2

ifeq ($(Debug), 1)
CFLAGS += -g -DDEBUG
endif

.PHONY = all clean

all:$(SERVER) $(CLIENT) $(TEST_0) $(TEST_1) $(TEST_2)

$(SERVER): shm_serv.o v4l2_driver.o
	$(CXX) -o $@ $^ $(INC_DIR) $(LDFLAGS)

$(CLIENT): shm_cli.o
	$(CXX) -o $@ $^ $(INC_DIR) $(LDFLAGS)

$(TEST_0): listing11_15.o
	$(CXX) -o $@ $^ $(INC_DIR) $(LDFLAGS)

$(TEST_1): listing11_18.o v4l2_driver.o
	$(CXX) -o $@ $^ $(INC_DIR) $(LDFLAGS)

$(TEST_2): listing11_16.o
	$(CXX) -o $@ $^ $(INC_DIR) $(LDFLAGS)

.cpp.o: 
	$(CXX) $(CFLAGS) -o $@ -c $^

clean:
	rm -f $(SERVER) $(CLIENT) $(TEST_0) $(TEST_1) $(TEST_2) *.o

