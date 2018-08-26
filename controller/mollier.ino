float getMollierVal(const float inTemp, const float inHumidity)
{
  float valRV;//ok1
  float valPS5;//ok2
  float valPT = 101325.262; //ok3


  valPS5 = 22120000 * (exp(((-7.691234564 * (1 - (273.15 + inTemp) / 647.3) - 26.08023696 * pow((1 - (273.15 + inTemp) / 647.3), 2) - 168.1706546 * pow((1 - (273.15 + inTemp) / 647.3), 3) + 64.23285504 * pow((1 - (273.15 + inTemp) / 647.3), 4) - 118.9646225 * pow((1 - (273.15 + inTemp) / 647.3), 5)) / ((273.15 + inTemp) / 647.3) / (1 + 4.16711732 * (1 - (273.15 + inTemp) / 647.3) + 20.9750676 * pow((1 - (273.15 + inTemp) / 647.3), 2)) - (1 - (273.15 + inTemp) / 647.3) / (1000000000 * pow((1 - (273.15 + inTemp) / 647.3), 2) + 6))));

  valRV = (inHumidity / 100 ) * valPS5;


  //            CALCULATION OF NB FUNCTION
  float n;
  float nh = inTemp;
  float nl = -273.15;
  float pw1;
  float ps ;
  for (int k = 1; k <= 50; k++)
  {
    n = (nh + nl) / 2;
    //alert(n);
    ps = calPSValue(n);

    pw1 = ps - 66.374 * (1 - 0.00112 * n) * (inTemp - n);

    if (pw1 < valRV)
    {
      nl = n;
    } else {
      nh = n;
    }

  }
  return n;
}

float calPSValue(const float val)
{
  float TS = val;
  float PS;

  if (TS >= 0)
  {
    PS = 22120000 * (exp(((-7.691234564 * (1 - (273.15 + TS) / 647.3) - 26.08023696 * pow((1 - (273.15 + TS) / 647.3), 2) - 168.1706546 * pow((1 - (273.15 + TS) / 647.3), 3) + 64.23285504 * pow((1 - (273.15 + TS) / 647.3), 4) - 118.9646225 * pow((1 - (273.15 + TS) / 647.3), 5)) / ((273.15 + TS) / 647.3) / (1 + 4.16711732 * (1 - (273.15 + TS) / 647.3) + 20.9750676 * pow((1 - (273.15 + TS) / 647.3), 2)) - (1 - (273.15 + TS) / 647.3) / (1000000000 * pow((1 - (273.15 + TS) / 647.3), 2) + 6))));
  } else {
    PS = 100 * exp(-20.947 * (273.15 / (273.15 + TS) - 1) - 3.56654 * log(273.15 / (273.15 + TS)) + 2.01889 * (1 - (273.15 + TS) / 273.15) + log(6.10714));
  }
  return PS;
}

