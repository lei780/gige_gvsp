

#include <cstdio>
#include <stdlib.h>
#include <cstdint>

#include <iostream>
#include <vector>
#include <array>

using namespace std;

uint8_t shm[2048];

int main()
{
    std::vector<uint8_t *> shared_regions; 

    //std::vector< std::array<uint8_t, 1518> > buf_packets; 
    std::vector<char *> buf_packets; 



    shared_regions.push_back(&shm[0]);
    shared_regions.push_back(&shm[256]);
    shared_regions.push_back(&shm[512]);
    shared_regions.push_back(&shm[768]);
    shared_regions.push_back(&shm[1024]);


    std::vector<uint8_t *>::iterator itr = shared_regions.begin();
    
    std::cout << "0x" << std::hex << (unsigned long)*itr++ << std::endl;
    std::cout << "-0x" << std::hex << (unsigned long)shared_regions[0] << std::endl;
    std::cout << "-0x" << std::hex << (unsigned long)shared_regions[1] << std::endl;

    std::cout << "0x" << std::hex << (unsigned long)*itr++ << std::endl;
    std::cout << "0x" << std::hex << (unsigned long)*itr++ << std::endl;
    std::cout << "0x" << std::hex << (unsigned long)*itr++ << std::endl;
    
    
    for(int i = 0; i < 10; i++){
        //std::array<uint8_t, 1518> *ptr = new array; 
        //char *ptr = new char[1518];
        //char *ptr = (char *)malloc(1518);

        //ptr[1000] = 'A'+i;
        //std::cout << "ptr[0]: " << std::dec << ptr[1000] << std::endl;

        buf_packets.push_back(new char[1518]);
    }

    for(int i = 0; i < 10; i++){
        char *ptr = buf_packets[i]; 
        ptr[1000] = 'A' + i;
    }

#if 1
    std::vector<char *>::iterator iter;
    int i = 0;

    //for(int i = 0; i < 10; i++)
    for(iter = buf_packets.begin(); iter != buf_packets.end(); iter++)
    {
        //std::array<uint8_t, 1518> ptr; 
        char *ptr = *iter;

        //ptr[1000] = 'A' + i++;
        //std::cout << "ptr[0]: " << ptr[0] << std::endl;
        std::cout << "ptr[0]: " << std::dec << ptr[1000] << std::endl;

        delete[] ptr;
        //delete[] *iter;
    }
#endif

}

