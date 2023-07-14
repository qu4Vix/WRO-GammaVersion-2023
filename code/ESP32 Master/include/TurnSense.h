#include <Arduino.h>

int8_t TurnSense(float* Distancias)
{
    float distanciaF;
    int angulo;
    int prevangulo = 359;
    float distancia = 0;
    float prevdistancia = *(Distancias+prevangulo);

    int angulomax;

    while (prevdistancia == 0)
        {
            prevdistancia = *(Distancias);
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
}