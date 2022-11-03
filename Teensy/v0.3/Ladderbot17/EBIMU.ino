
int EBimuAsciiParser(float * item, int number_of_item)
{

  int n, i;
  int rbytes;
  char *addr;
  int result = 0;

  rbytes = Serial2.available();

  for (n = 0; n < rbytes; n++)
  {
    sbuf[sbuf_cnt] = Serial2.read();

    if (sbuf[sbuf_cnt] == 0x0a)
    {
      addr = strtok(sbuf, ",");
      for (i = 0; i < number_of_item; i++)
      {
        item[i] = atof(addr);

        addr = strtok(NULL, ",");

      }

      result = 1;
    }
    else if (sbuf[sbuf_cnt] == '*')
    { sbuf_cnt = -1;
    }

    sbuf_cnt++;
    if (sbuf_cnt >= SBUF_SIZE) sbuf_cnt = 0;

  }

  return result;

}

void EBimuCommand(String sb1) {
  byte buffer[sb1.length() + 1];

  sb1.getBytes(buffer, sb1.length() + 1);

  for (int i = 0; i < sb1.length() + 1; i++) {
    // AudioNoInterrupts();//////////////////////////////////////////////Important!
    Serial2.write(buffer[i]);
    // AudioInterrupts();
  }
}
