/** function for approximate wet temperature by mollier chart to specific humidity
 *  
**/
float wetTemp(const float inTemp, const float inHumid, const float outHumid) {
  float wetbulb = mollierTemp(inTemp, inHumid);
  float tailTemp = mollierTemp(inTemp, outHumid);
  return wetbulb + (inTemp - tailTemp);
}
/** function to wet blub temperature
 *  
 */
float mollierTemp(const float inTemp, const float inHumidity) {
  float valRV = (inHumidity / 100 ) * calPSValue(inTemp);

  //            CALCULATION OF NB FUNCTION
  float n;
  float nh = inTemp;
  float nl = -273.15;
  float pw1;
  float ps ;
  for (int k = 1; k <= 30; k++)
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

float calPSValue(const float val) {
  float f1 = (1 - (273.15 + val) / 647.3);
  if (val >= 0)
  {
    return 22120000 * (exp(((-7.691234564 * f1
                             - 26.08023696 * pow(f1, 2)
                             - 168.1706546 * pow(f1, 3)
                             + 64.23285504 * pow(f1, 4)
                             - 118.9646225 * pow(f1, 5)) /
                            ((273.15 + val) / 647.3) /
                            (1 + 4.16711732 * f1 + 20.9750676 * pow(f1, 2)) - f1 / (1000000000 * pow(f1, 2) + 6))));
  } else {
    return 100 * exp(-20.947 * (273.15 / (273.15 + val) - 1) - 3.56654 * log(273.15 / (273.15 + val)) + 2.01889 * (1 - (273.15 + val) / 273.15) + 0.78584);
  }
}
