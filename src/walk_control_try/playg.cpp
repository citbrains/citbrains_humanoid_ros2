#include <chrono>
#include <iostream>
#include <cstdint>

using namespace std;

int main(){
    auto start = chrono::system_clock::now();
    for(int i = 0;i < 2e14;++i){

    }
    auto end = chrono::system_clock::now();
    chrono::duration<std::chrono::microseconds> elaps = end - start;
}