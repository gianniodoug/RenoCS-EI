#ifndef Sercomm_h
#define Sercomm_h

// Routine de recuperation d'un caractere
#include "Arduino.h"

char GetChar(char def);     // recuperation d'un caractere sur la liaison série
int GetInt(int def);        // routine de récupération d'un nombre entier sur la liaison série
long int GetLong(long def); // routine de récupération d'un nombre entier long (32 bits) sur la liaison série
char* GetString();          // Routine de recuperation d'une chaine de caractères
void write_i8(char num);    // renvoi de valeur binaire 8 bits sur la liaison série
void write_i16(int num);    // renvoi de valeur binaire 16 bits sur la liaison série
void write_i32(long num);   // renvoi de valeur binaire 32 bits sur la liaison série
void RetAcquitSimpl();      // routine renvoyant le message d'acquitement simple (OK ou OBstacle)
void RetAcquit(__FlashStringHelper* strng);
void RetAcquit(__FlashStringHelper* strng,int val);
void RetAcquit(__FlashStringHelper* strng,int val1, int val2 );
void RetAcquit(__FlashStringHelper* strng,long val1, long val2 );
void RetAcquit(__FlashStringHelper* strng,int val1,int val2,int val3 );
void RetAcquit(__FlashStringHelper* strng,int val1,int val2,int val3,int val4 );
void Ret2l(long v1,long v2);
void Ret2i(int v1,int v2);
void Ret4i(int v1,int v2,int v3,int v4);
void Ret2li(long v1,int v2);
//void Ret4l(long v1,long v2,long v3,long v4);

#endif