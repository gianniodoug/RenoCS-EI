//////////////////////////////////////////////////////////////////////////
//
// LES FONCTIONS DE COMMUNICATION
//
//////////////////////////////////////////////////////////////////////////

#include "Sercomm.h"
extern char commode ;
extern char feedback ;
extern bool obst ; 

char dispstring[33] ;   // chaine de retour de commande

// Routine de recuperation d'un caractere
char GetChar(char def)
{   
    if (Serial.available()>0)  // liaison série vide 
      return(Serial.read());
    else
      return(def) ;
}

// routine de récupération d'un nombre entier sur la liaison série
// le codage sera ascii ou binaire selon la variable 'commode'
int GetInt(int def)
{   
    int tmp ;
    char c ;
    if (commode==0) {
      c=Serial.peek() ;
      if (c==' ') {Serial.read(); c=Serial.peek() ;}
      if ((c=='-') || ((c>='0') && (c<='9'))) 
        return(Serial.parseInt()); // on recupere la commande du moteur
      else
        return(def) ; }  // renvoie la valeur par défaut 
    else {
      Serial.readBytes((char*)&tmp,2);
      return(tmp) ;
    }
    
}

// routine de récupération d'un nombre entier long (32 bits) sur la liaison série
// le codage sera ascii ou binaire selon la variable 'commode'
long int GetLong(long def)
{
    long int tmp ;
    char c;
    if (commode==0) {
      c=Serial.peek() ;
      if (c==' ') {Serial.read(); c=Serial.peek() ;}
      if ((c=='-') || ((c>='0') && (c<='9'))) 
      {
        tmp=Serial.parseInt() ;
        return(tmp); // on recupere la commande du moteur
      }
      else
        return(def) ; }  // renvoie la valeur par défaut
    else {
      Serial.readBytes((char*)&tmp,4);
      return(tmp) ;
    }
}

// Routine de recuperation d'une chaine de caractères
char* GetString()
{  
    char ind=0; 
    char c=0;
    bool eol=false;
    while (eol==false)  // liaison série vide    Serial.available()>0 && 
    {
      c=Serial.read();
      if (c=='\n')
        eol=true ;
      else
        dispstring[ind++]=c;
    }
      dispstring[ind]=0 ;
      return(dispstring) ;
} 


// bibliothèque de renvoi de valeurs binaires sur la liaison série
inline void write_i8(char num)
{ Serial.write(num); }

void write_i16(int num)
{ Serial.write((uint8_t*)&num, 2); }

void write_i32(long num)
{ Serial.write((uint8_t*)&num, 4); }

// routine renvoyant le message d'acquitement simple (OK ou OBstacle)
void RetAcquitSimpl()
{
    if (feedback==1) 
      if (obst==false) Serial.println(F("OK")); 
      else Serial.println(F("OB"));
}

void Acquit()
{
    if (feedback==1) 
      if (obst==false) Serial.println(F("OK")); 
      else Serial.println(F("OB"));
}

// routine renvoyant un message d'acquitement et une chaine de caractère
void RetAcquit(__FlashStringHelper* strng)
{
    Acquit() ;
    if (feedback==2)
    {
      Serial.println(strng); 
    }  
}

// routine renvoyant un message d'acquitement, une chaine de caractère et 1 valeur entière
void RetAcquit(__FlashStringHelper* strng,int val)
{
    Acquit() ;
    if (feedback==2)
    {
      Serial.print(strng); 
      Serial.println(val); 
    }  
}

// routine renvoyant un message d'acquitement, une chaine de caractère et 2 valeurs entières
void RetAcquit(__FlashStringHelper* strng,int val1,int val2 )
{
    Acquit() ;
    if (feedback==2)
    {
      Serial.print(strng); 
      Serial.print(val1); 
      Serial.print(F(" ")); 
      Serial.println(val2); 
    }  
}

// routine renvoyant un message d'acquitement, une chaine de caractère et 2 valeurs long
void RetAcquit(__FlashStringHelper* strng,long  val1,long  val2 )
{
    Acquit() ;
    if (feedback==2)
    {
      Serial.print(strng); 
      Serial.print(val1); 
      Serial.print(F(" ")); 
      Serial.println(val2); 
    }  
}

// routine renvoyant un message d'acquitement, une chaine de caractère et 3 valeurs entières
void RetAcquit(__FlashStringHelper* strng,int val1,int val2, int val3 )
{
    Acquit() ;
    if (feedback==2)
    {
      Serial.print(strng); 
      Serial.print(val1); 
      Serial.print(F(" ")); 
      Serial.print(val2); 
      Serial.print(F(" ")); 
      Serial.println(val3); 
    }  
}

// routine renvoyant un message d'acquitement, une chaine de caractère et 4 valeurs entières
void RetAcquit(__FlashStringHelper* strng,int val1,int val2, int val3, int val4 )
{
    Acquit() ;
    if (feedback==2)
    {
      Serial.print(strng); 
      Serial.print(val1); 
      Serial.print(F(" ")); 
      Serial.print(val2); 
      Serial.print(F(" ")); 
      Serial.print(val3); 
      Serial.print(F(" ")); 
      Serial.println(val4); 
    }  
}

// routine renvoyant 2 valeurs long
void Ret2l(long v1,long v2)
{
  if (commode==2)
  {  write_i32(v1); write_i32(v2); }
  else
  { Serial.print(v1);
    Serial.print(" ");
    Serial.println(v2); }
}

// routine renvoyant 2 valeurs int
void Ret2i(int v1,int v2)
{
  if (commode==2)
  {  write_i16(v1); write_i16(v2); write_i16(0);  write_i16(0); }
  else
  { Serial.print(v1);
    Serial.print(" ");
    Serial.println(v2); }
}

// routine renvoyant 4 valeurs int
void Ret4i(int v1, int v2, int v3, int v4)
{
  if (commode==2)
  {  write_i16(v1); write_i16(v2); write_i16(v3);  write_i16(v4); }
  else
  { Serial.print(v1);
    Serial.print(" ");
    Serial.print(v2);
    Serial.print(" ");
    Serial.print(v3);
    Serial.print(" ");
    Serial.println(v4); }
}

// routine renvoyant 1 long et 1 int
void Ret2li(long v1,int v2)
{
  if (commode==2)
  {  write_i32(v1); write_i16(v2);  write_i16(0);  }
  else
  { Serial.print(v1);
    Serial.print(" ");
    Serial.println(v2); }
}

// routine renvoyant 4 valeurs long
void Ret4l(long v1,long v2,long v3,long v4)
{
  if (commode==2)
  {  write_i32(v1); write_i32(v2); write_i32(v3); write_i32(v4); }
  else
  { Serial.print(v1);
    Serial.print(" ");
    Serial.print(v2);
    Serial.print(" ");
    Serial.print(v3);
    Serial.print(" ");
    Serial.println(v4); }
}

