#include "main.h"

int main(){
    Serial.begin(115200);
    while(1){
        Serial.println(2);
        while(1){
            asm("");
        }
    }
    return 0;
}
