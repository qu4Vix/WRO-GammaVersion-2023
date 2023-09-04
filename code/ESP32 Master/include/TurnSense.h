#include <Arduino.h>

int8_t TurnSense(uint16_t* Distancias, int* Millis)
{
    int16_t medidasd;
    uint16_t medidasi;

    uint16_t rep;

    uint16_t id = 80;
    while (rep < 10)
    {
        if (*(Millis+id) < (millis()+500))
        {
            if (*(Distancias+id) > 1000)
            {
                medidasd++;
            }
        }
        if (id == 100)
        {
            id = 79;
            rep++;
        }
        id++;
    }
    
    uint16_t ii = 260;
    while (rep < 10)
    {
        if (*(Millis+ii) < (millis()+500))
        {
            if (*(Distancias+ii) > 1000)
            {
                medidasi++;
            }
        }
        if (ii == 280)
        {
            ii = 259;
            rep++;
        }
        ii++;
    }

    if (medidasd > medidasi && medidasd > 3)
    {
        //derecha
        return -1; //1
    }
    else if (medidasi > medidasd && medidasi > 3)
    {
        //izquierda
        return 1; //-1
    }
    else{return 0;}
}