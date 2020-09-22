# IoT_estacMeteo
Sistema baseado em uma estação meteorológica com integração IoT que auxilia no manejo de irrigação.

O projeto em questão se refere a um sistema com base em uma estação meteorológica com integração IoT para auxiliar o manejo de irrigação, calculando a evapotranspiração da cultura (ETc) de acordo com o método padrão considerado pela Organização das Nações Unidas para Alimentação e Agricultura (FAO). 
Assim, internamente o sistema funciona com um ESP32 atuando como microcontrolador para a coleta dos dados por meio de diversos sensores e para o processamento e transmissão tanto desses dados quanto dos relativos ao manejo de irrigação (especialmente, a ETc). Há a conexão com a IBM Cloud (IBM Watson IoT Platform), que atua como servidor. Por fim, é realizada conexão, também, com o Node-RED (para o tratamento dos dados e visualização em um host particular) e com o Cloudant (para o armazenamento dos dados em nuvem).
