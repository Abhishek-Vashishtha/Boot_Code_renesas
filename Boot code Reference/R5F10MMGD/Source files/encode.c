unsigned int encode(unsigned long con)
{
  unsigned char a,b,*ptr;
  unsigned int c;
  ptr = (unsigned char *)&con;
  a = *(ptr+2);
  a += *(ptr+1);
  a += *ptr;
 
  b = *(ptr+3);
  b += *(ptr+1);
  b += *ptr;
  
  a ^= *(ptr+1);
  a += *(ptr+3);
   
  b ^= *(ptr+0);
  b += *(ptr+2);
 
  c  = ( a>>4 | a<<4);
  c <<= 8;
  b = (b>>4 |  b<<4);
 return(c|b);
}