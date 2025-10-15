#include <stdint.h>
#include <stdio.h>

int main() {
    //Declaracion de Arreglos
    uint32_t LDR1[150]={},LDR2[150]={},buffer[300]={};

    //Punteros
    uint32_t* ptr1 = &buffer[0];
    uint32_t* ptr2 = &buffer[1];

    //Variable para el ciclo for
    uint16_t i=0;

    //Simulamos los valores metiendo numeros del 0 al 299 en el buffer
    for(i=0;i<300;i++) {
        buffer[i]=i;
    }

    //Separamos los valores del buffer en dos arreglos
    for(i=0;i<150;i++) {
        LDR1[i]=*ptr1;
        LDR2[i]=*ptr2;
        ptr1+=2;
        ptr2+=2;
    }

    //Imprimimos los arreglos de 150 valores cada uno
    printf("%s","Valores De LDR1 \n");
    for(i=0;i<150;i++) {

        printf("%i ",LDR1[i]);
    }

    printf("\n");

    printf("%s","Valores De LDR2 \n");
    for(i=0;i<150;i++) {
        printf("%i ",LDR2[i]);
    }

    return 0;
}
