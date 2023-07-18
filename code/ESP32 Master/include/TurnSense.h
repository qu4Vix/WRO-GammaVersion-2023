#include <Arduino.h>

int8_t TurnSense(float* Distancias)
{
    float distanciaF;
    int angulo = 358;
    int prevangulo = 359;
    float distancia = *(Distancias+angulo);
    float prevdistancia = *(Distancias+prevangulo);

    int angulomax;

    while (prevdistancia == 0)
    {
        prevdistancia = *(Distancias+prevangulo);
        distanciaF = *(Distancias+prevangulo);
    }

    while (distancia == 0)
    {
        distancia = *(Distancias+angulo);
    }

    while (abs(distancia-prevdistancia) < 20)
    {
        angulo = prevangulo - 1;
        distancia = *(Distancias+angulo);
        while (distancia == 0)
        {
            angulo--;
            distancia = *(Distancias+angulo);
        } 
    }

    while (*(Distancias+270) == 0);

    if ((tan(359-prevangulo)*distanciaF) > *(Distancias+270))
    {
        //Girar izquierda
        return 1; //o -1
    }
}